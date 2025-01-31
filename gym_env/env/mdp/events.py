from __future__ import annotations

import torch
from typing import TYPE_CHECKING, Literal

import carb
import omni.physics.tensors.impl.api as physx

import omni.isaac.lab.sim as sim_utils
import omni.isaac.lab.utils.math as math_utils
from omni.isaac.lab.actuators import ImplicitActuator
from omni.isaac.lab.assets import Articulation, DeformableObject, RigidObject
from omni.isaac.lab.managers import EventTermCfg, ManagerTermBase, SceneEntityCfg
from omni.isaac.lab.terrains import TerrainImporter

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedEnv



def randomize_rigid_body_mass(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    mass_distribution_params: tuple[float, float],
    operation: Literal["add", "scale", "abs"],
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
    recompute_inertia: bool = True,
):
    """Randomize the mass of the bodies by adding, scaling, or setting random values.

    This function allows randomizing the mass of the bodies of the asset. The function samples random values from the
    given distribution parameters and adds, scales, or sets the values into the physics simulation based on the operation.

    If the ``recompute_inertia`` flag is set to ``True``, the function recomputes the inertia tensor of the bodies
    after setting the mass. This is useful when the mass is changed significantly, as the inertia tensor depends
    on the mass. It assumes the body is a uniform density object. If the body is not a uniform density object,
    the inertia tensor may not be accurate.

    .. tip::
        This function uses CPU tensors to assign the body masses. It is recommended to use this function
        only during the initialization of the environment.
    """
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject | Articulation = env.scene[asset_cfg.name]

    # resolve environment ids
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device="cpu")
    else:
        env_ids = env_ids.cpu()

    # resolve body indices
    if asset_cfg.body_ids == slice(None):
        body_ids = torch.arange(asset.num_bodies, dtype=torch.int, device="cpu")
    else:
        body_ids = torch.tensor(asset_cfg.body_ids, dtype=torch.int, device="cpu")

    # get the current masses of the bodies (num_assets, num_bodies)
    masses = asset.root_physx_view.get_masses()

    # apply randomization on default values
    # this is to make sure when calling the function multiple times, the randomization is applied on the
    # default values and not the previously randomized values
    masses[env_ids[:, None], body_ids] = asset.data.default_mass[env_ids[:, None], body_ids].clone()

    # sample from the given range
    # note: we modify the masses in-place for all environments
    #   however, the setter takes care that only the masses of the specified environments are modified
    masses = _randomize_prop_by_op(
        masses, mass_distribution_params, env_ids, body_ids, operation=operation, distribution=distribution
    )

    # set the mass into the physics simulation
    asset.root_physx_view.set_masses(masses, env_ids)

    # recompute inertia tensors if needed
    if recompute_inertia:
        # compute the ratios of the new masses to the initial masses
        ratios = masses[env_ids[:, None], body_ids] / asset.data.default_mass[env_ids[:, None], body_ids]
        # scale the inertia tensors by the the ratios
        # since mass randomization is done on default values, we can use the default inertia tensors
        inertias = asset.root_physx_view.get_inertias()
        if isinstance(asset, Articulation):
            # inertia has shape: (num_envs, num_bodies, 9) for articulation
            inertias[env_ids[:, None], body_ids] = (
                asset.data.default_inertia[env_ids[:, None], body_ids] * ratios[..., None]
            )
        else:
            # inertia has shape: (num_envs, 9) for rigid object
            inertias[env_ids] = asset.data.default_inertia[env_ids] * ratios
        # set the inertia tensors into the physics simulation
        asset.root_physx_view.set_inertias(inertias, env_ids)






def _randomize_prop_by_op(
    data: torch.Tensor,
    distribution_parameters: tuple[float | torch.Tensor, float | torch.Tensor],
    dim_0_ids: torch.Tensor | None,
    dim_1_ids: torch.Tensor | slice,
    operation: Literal["add", "scale", "abs"],
    distribution: Literal["uniform", "log_uniform", "gaussian"],
) -> torch.Tensor:
    """Perform data randomization based on the given operation and distribution.

    Args:
        data: The data tensor to be randomized. Shape is (dim_0, dim_1).
        distribution_parameters: The parameters for the distribution to sample values from.
        dim_0_ids: The indices of the first dimension to randomize.
        dim_1_ids: The indices of the second dimension to randomize.
        operation: The operation to perform on the data. Options: 'add', 'scale', 'abs'.
        distribution: The distribution to sample the random values from. Options: 'uniform', 'log_uniform'.

    Returns:
        The data tensor after randomization. Shape is (dim_0, dim_1).

    Raises:
        NotImplementedError: If the operation or distribution is not supported.
    """
    # resolve shape
    # -- dim 0
    if dim_0_ids is None:
        n_dim_0 = data.shape[0]
        dim_0_ids = slice(None)
    else:
        n_dim_0 = len(dim_0_ids)
        if not isinstance(dim_1_ids, slice):
            dim_0_ids = dim_0_ids[:, None]
    # -- dim 1
    if isinstance(dim_1_ids, slice):
        n_dim_1 = data.shape[1]
    else:
        n_dim_1 = len(dim_1_ids)

    # resolve the distribution
    if distribution == "uniform":
        dist_fn = math_utils.sample_uniform
    elif distribution == "log_uniform":
        dist_fn = math_utils.sample_log_uniform
    elif distribution == "gaussian":
        dist_fn = math_utils.sample_gaussian
    else:
        raise NotImplementedError(
            f"Unknown distribution: '{distribution}' for joint properties randomization."
            " Please use 'uniform', 'log_uniform', 'gaussian'."
        )
    # perform the operation
    if operation == "add":
        data[dim_0_ids, dim_1_ids] += dist_fn(*distribution_parameters, (n_dim_0, n_dim_1), device=data.device)
    elif operation == "scale":
        data[dim_0_ids, dim_1_ids] *= dist_fn(*distribution_parameters, (n_dim_0, n_dim_1), device=data.device)
    elif operation == "abs":
        data[dim_0_ids, dim_1_ids] = dist_fn(*distribution_parameters, (n_dim_0, n_dim_1), device=data.device)
    else:
        raise NotImplementedError(
            f"Unknown operation: '{operation}' for property randomization. Please use 'add', 'scale', or 'abs'."
        )
    return data