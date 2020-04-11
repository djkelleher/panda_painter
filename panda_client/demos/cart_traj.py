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

    set_workspace_req = {
        "service": "/set_workspace",
        "minX":0.15, "minY":-0.55, "minZ":0.15,
        "maxX":0.75, "maxY":0.55, "maxZ":0.55
    }
    service_calls.append(set_workspace_req)
    
    # create a CartTraj service request message in dictionary format.
    box = []

    box.append({"x_pos": 0.3, "y_pos": 0.5, "z_pos": 0.3, "x_rot":pi})
    
    box.append({"x_pos": 0.5, "y_pos": 0.5, "z_pos": 0.3, "x_rot":pi})

    box.append({"x_pos": 0.5, "y_pos": -0.5, "z_pos": 0.3, "x_rot":pi})

    box.append({"x_pos": 0.3, "y_pos": -0.5, "z_pos": 0.3, "x_rot":pi})

    box.append({"x_pos": 0.3, "y_pos": 0.5, "z_pos": 0.3, "x_rot":pi})


    xz_extension = []

    xz_extension.append({"x_pos": 0.75, "z_pos": 0.5, "x_rot":pi})

    xz_extension.append({"x_pos": 0.2, "z_pos": 0.5, "x_rot":pi})

    xz_extension.append({"x_pos": 0.2, "z_pos": 0.75, "x_rot":pi})


    y_extension = []

    y_extension.append({"x_pos": 0.5, "y_pos": 0.0, "z_pos": 0.5, "x_rot":pi})

    y_extension.append({"x_pos": 0.5, "y_pos": 0.55, "z_pos": 0.5, "x_rot":pi})

    y_extension.append({"x_pos": 0.5, "y_pos": -0.55, "z_pos": 0.5, "x_rot":pi})

    # tell rosbridge to call /execute_cart_traj service with our task_dict.
    service_calls.append({"service": "/execute_cart_traj", "waypoints": box})
    # finish in home position.
    service_calls.append({"service": "/goto_home"})
    # add cart_traj_demo to the database.
    panda_client.add_task("cart_traj_demo", service_calls)
    # execute task.
    panda_client.execute_task("cart_traj_demo")