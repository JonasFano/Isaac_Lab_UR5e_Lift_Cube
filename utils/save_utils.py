import time
import os

class SaveUtils:
    @staticmethod
    def prepare_data_saving(save_data, data_path, data_file_path):
        """
        Changes the file name if it already exists and adds headers to the new 3 files where the joint positions will be saved.

        Returns
        - data_file_path (str): Path with the changed file name if it was already existent.
        """
        if save_data:
            if os.path.exists(data_file_path + "_pick_up.csv"):
                i = 2
                while os.path.exists(data_file_path + "_pick_up.csv"): 
                    data_file_path = os.path.join(data_path, "demonstration_" + str(i))
                    i += 1
                
            print("Demonstration is saved to file: ", data_file_path)

            # Add headers if the file does not exist
            if not os.path.exists(data_file_path + "_pick_up.csv"):
                with open(data_file_path + "_pick_up.csv", "w") as myfile:
                    headers = "timestamp actual_q_0 actual_q_1 actual_q_2 actual_q_3 actual_q_4 actual_q_5 actual_q_6 actual_q_7\n"
                    myfile.write(headers)
                with open(data_file_path + "_place.csv", "w") as myfile:
                    headers = "timestamp actual_q_0 actual_q_1 actual_q_2 actual_q_3 actual_q_4 actual_q_5 actual_q_6 actual_q_7\n"
                    myfile.write(headers)
                with open(data_file_path + "_home.csv", "w") as myfile:
                    headers = "timestamp actual_q_0 actual_q_1 actual_q_2 actual_q_3 actual_q_4 actual_q_5 actual_q_6 actual_q_7\n"
                    myfile.write(headers)
        return data_file_path

    @staticmethod
    def save(save_data, robot, dmp_nr, data_file_path):
        """
        Saves joint positions with timestamps in 3 separate files depending on the current state.
        """
        if save_data:
            # Get the current Unix timestamp (in seconds)
            timestamp = time.time()  # Unix timestamp

            # Convert the joint positions tensor to a Python list
            joint_positions_list = robot.data.joint_pos.clone().tolist()[0]

            # Convert the list of joint positions to a space-separated string (without brackets)
            # joint_positions_str = ' '.join(f"{val:.6f}" for val in joint_positions_list)  # Format each value to 6 decimal places
            joint_positions_str = ' '.join(f"{val:.6f}" for val in joint_positions_list)


            # Create the final entry with the Unix timestamp and joint positions
            entry = f"{timestamp} {joint_positions_str}\n"  # Format as timestamp followed by joint positions

            # Append the entry to the correct file based on dmp_nr
            if dmp_nr == 1:
                with open(data_file_path + "_pick_up.csv", "a") as myfile:
                    myfile.write(entry)  # Write the timestamp and joint positions to the file
            elif dmp_nr == 2:
                with open(data_file_path + "_place.csv", "a") as myfile:
                    myfile.write(entry)  # Write the timestamp and joint positions to the file
            else:
                with open(data_file_path + "_home.csv", "a") as myfile:
                    myfile.write(entry)  # Write the timestamp and joint positions to the file


    @staticmethod
    def save_w_gripper_correction(save_data, robot, dmp_nr, data_file_path):
        """
        Saves joint positions with timestamps in 3 separate files depending on the current state.
        """
        if save_data:
            # Get the current Unix timestamp (in seconds)
            timestamp = time.time()  # Unix timestamp

            # Convert the joint positions tensor to a Python list
            joint_positions_list = robot.data.joint_pos.clone().tolist()[0]
            joint_pos_des_list = robot.data.joint_pos_target.tolist()[0]

            # Convert the list of joint positions to a space-separated string (without brackets)
            # joint_positions_str = ' '.join(f"{val:.6f}" for val in joint_positions_list)  # Format each value to 6 decimal places
            joint_positions_str = ' '.join(f"{val:.6f}" for val in joint_positions_list[:6]) + ' ' + \
                              ' '.join(f"{val:.6f}" for val in joint_pos_des_list[6:])


            # Create the final entry with the Unix timestamp and joint positions
            entry = f"{timestamp} {joint_positions_str}\n"  # Format as timestamp followed by joint positions

            # Append the entry to the correct file based on dmp_nr
            if dmp_nr == 1:
                with open(data_file_path + "_pick_up.csv", "a") as myfile:
                    myfile.write(entry)  # Write the timestamp and joint positions to the file
            elif dmp_nr == 2:
                with open(data_file_path + "_place.csv", "a") as myfile:
                    myfile.write(entry)  # Write the timestamp and joint positions to the file
            else:
                with open(data_file_path + "_home.csv", "a") as myfile:
                    myfile.write(entry)  # Write the timestamp and joint positions to the file
