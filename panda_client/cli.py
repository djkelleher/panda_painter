from panda_client import PandaClient
import websocket
import json
import sys

def cli():
    panda_client = PandaClient(ros_box_address='127.0.0.1', port=9090)

    command = None
    while command != 'q':
        call_service_msg = None
        # Create a list for service requests. Add scale_limits_req so each request gets scaled.
        command = input("[panda] Type 'help' to show commands. Type 'q' to quit.\nEnter a command: ")
        command = str(command).lower()
        if command == 'help':
            print("\n"
                "commands:\n\n"
                "'list_tasks': list the names of all tasks currently stored in the database.\n\n"
                "'print_task': print a sored task to the terminal.\n\n"
                "'execute_task task1_name taskn_name': execute a task or tasks stored in the database.\n\n"
                "'remove_task task1_name taskn_name': remove a task or tasks from the database.\n\n"
                "'gripper_pos width speed': send a gripper position and velocity command.\n\n"
                "'joint_pos joint1 joint2 joint3 joint4 joint5 joint6 joint7': send a joint position goal.\n\n"
                "'cart_pos x_pos y_pos z_pos x_rot y_rot z_rot': send a cartesian goal pose request.\n\n"
                "'home': move the arm to the home position.\n"
            )
        elif command == "list_tasks":
            panda_client.list_tasks()
        elif "print_task" in command:
            task_name = command.split()[1]
            panda_client.print_task(task_name)
        elif "execute_task" in command:
            tasks = command.split()[1:]
            if len(tasks) > 1:
                print("Tasks will be executed in the following order: {}. Press 'y' to confirm, or 'n' to cancel.".format(tasks))
                conformation = input()
                while conformation not in ('y', 'n', 'q'):
                    print("Either 'y' 'n' or 'q' must be entered.")
                    conformation = input()
                if conformation == 'n':
                    continue
                if conformation == 'q':
                    break
            panda_client.execute_task(tasks)
        elif "remove_task" in command:
            tasks = command.split()[1:]
            for task in tasks:
                panda_client.remove_task(task)
        elif "gripper_pos" in command:
            split_command = command.split()
            if len(split_command) != 3:
                print("[Error] args should be: width speed")
                continue
            width = float(split_command[1])
            speed = float(split_command[2])
            # tell rosbridge to call /goto_gripper_pos service with num_req.
            call_service_msg = {
                "service": "/goto_gripper_pos",
                "width": width,
                "speed": speed
            }
        elif "joint_pos" in command:
            split_command = command.split()
            if len(split_command) != 8:
                print("[Error] args should be: joint1 joint2 joint3 joint4 joint5 joint6 joint7")
                continue
            # create a JointTraj service request message.
            goal_position = []
            goal_position.append(float(split_command[1]))
            goal_position.append(float(split_command[2]))
            goal_position.append(float(split_command[3]))
            goal_position.append(float(split_command[4]))
            goal_position.append(float(split_command[5]))
            goal_position.append(float(split_command[6]))
            goal_position.append(float(split_command[7]))
            # tell rosbridge to call /goto_joint_pos service with our goal list.
            call_service_msg = {
                "service": "/goto_joint_pos",
                "joint_goal_list": goal_position
            }
        elif "cart_pos" in command:
            split_command = command.split()
            if len(split_command) != 7:
                print(" [Error] args should be: x_pos y_pos z_pos x_rot y_rot z_rot")
                continue
            # create a CartTraj service request message in dictionary format.
            goal_pos = {
                "x_pos": float(split_command[1]), "y_pos": float(split_command[2]), "z_pos": float(split_command[3]),
                "x_rot": float(split_command[4]), "y_rot": float(split_command[5]), "z_rot": float(split_command[6])
            }
            # tell rosbridge to call /goto_cart_pos service with our task_dict.
            call_service_msg = {
                "service": "/goto_cart_pos",
                "waypoint": goal_pos
            }
        elif "home" in command:
            # tell rosbridge to call /goto_home service.
            call_service_msg = {
                "service": "/goto_home"
            }
        elif command == "set_workspace":
            split_command = command.split()
            if len(split_command) != 7:
                print(" [Error] args should be: minX minY minZ maxX maxY maxZ")
                continue
            # create SetWorkspace service request message in dictionary format.
            call_service_msg = {
                "service": "/set_workspace",
                "minX": float(split_command[1]), "minY": float(split_command[2]), "minZ": float(split_command[3]),
                "maxX": float(split_command[4]), "maxY": float(split_command[5]), "maxZ": float(split_command[6])
            }
        elif command is not 'q':
            print("Invalid command: {}".format(command))
        else:
            print("[panda] exiting.")
        
        if call_service_msg is not None:
            scale_limits_req = {
                "service": "/scale_limits",
                "velocity_scale_factor": 0.1,
                "acceleration_scale_factor": 0.1
            }
            service_calls = [scale_limits_req, call_service_msg]
            panda_client.add_task("cmd_line_task", service_calls)
            panda_client.execute_task("cmd_line_task")

if __name__=="__main__":
    cli()