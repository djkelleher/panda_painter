#!/usr/bin/env python3

from panda_client  import PandaClient
from math import pi

if __name__ == '__main__':

    # connect to the machine running ROS.
    panda_client = PandaClient(ros_box_address='127.0.0.1', port=9090)

    # Create a list for service requests.
    service_calls = []

    # constrain velocity and acceleration.
    scale_limits_req = {
        "service": "/scale_limits",
        "velocity_scale_factor": 0.1,
        "acceleration_scale_factor": 0.1
    }
    service_calls.append(scale_limits_req)

    goal_pose1 = {"x_pos": 0.2, "y_pos": 0.5, "z_pos": 0.3, "x_rot":pi}
    service_calls.append({"service": "/goto_cart_pos", "waypoint": goal_pose1})

    goal_pose2 = {"x_pos": 0.5, "y_pos": 0.5, "z_pos": 0.3, "x_rot":pi}
    service_calls.append({"service": "/goto_cart_pos", "waypoint": goal_pose2})

    goal_pose3 = {"x_pos": 0.5, "y_pos": -0.5, "z_pos": 0.3, "x_rot":pi}
    service_calls.append({"service": "/goto_cart_pos", "waypoint": goal_pose3})

    goal_pose4 = {"x_pos": 0.2, "y_pos": -0.5, "z_pos": 0.3, "x_rot":pi}
    service_calls.append({"service": "/goto_cart_pos", "waypoint": goal_pose4})

    goal_pose5 = {"x_pos": 0.2, "y_pos": 0.5, "z_pos": 0.3, "x_rot":pi}
    service_calls.append({"service": "/goto_cart_pos", "waypoint": goal_pose5})

    # finish in home position.
    service_calls.append({"service": "/goto_home"})
    # add cart_traj_demo to the database.
    panda_client.add_task("cart_pose_demo", service_calls)
    # execute task.
    panda_client.execute_task("cart_pose_demo")