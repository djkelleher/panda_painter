#!/usr/bin/env python3

from panda_client  import PandaClient
import cv2
import sys
from math import pi


if __name__=="__main__":
    # connect to the machine running ROS.
    panda_client = PandaClient(ros_box_address='127.0.0.1', port=9090)
    
    # Create a list for all the painting task service messages.
    painting_service_calls = []

    change_collision_behavior = {
        "service": "/change_collision_behavior",
        "add_val": 7
    }
    painting_service_calls.append(change_collision_behavior)

    # start in home position.
    painting_service_calls.append({"service": "/goto_home"})

    # create the canvas.
    painting_canvas = {
        "x_pos":0.5, "y_pos":0.0, "z_pos":0.3, #location
        "x_dim":0.4, "y_dim":0.4, "z_dim":0.02 #dimensions
    }
    # add canvas to service request.
    set_canvas_srv_req = {
        "service": "/set_canvas",
        "canvas": painting_canvas
    }
    painting_service_calls.append(set_canvas_srv_req)

    # get picture for painting.
    picture_file = "../images/pig.png"

    # load input image
    image = cv2.imread(picture_file,0)
    if image is None:
        print("Unable to load: ", picture_file)
        sys.exit(1)

    # convert the photo into simple lines
    thresh = cv2.threshold(image,127,255,0)[1]
    contours = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]

    # image shape
    height, width = image.shape
    cx = width/2
    cy = height/2
    # maps points on image to x,y of canvas.
    scale = min(painting_canvas["x_dim"], painting_canvas["y_dim"]) / max(height, width)

    # waypoint planning is done with the last link of the arm rather than the end effector.
    # Add an offset in the z direction to compensate for the size of the end effector.
    path_height = painting_canvas["z_pos"]+0.15
    # up_heigh = how much to raise the arm when it is transitioning between lines.
    up_height = path_height+0.05
    paint_location = {"x_pos":painting_canvas["x_pos"]-0.15, "y_pos":painting_canvas["y_pos"]+0.275}

    # get waypoints for each contour.
    for i in range(len(contours) - 1):
        # store all waypoints in a list.
        waypoint_list = []
        # get paint
        waypoint_list.append({"x_pos":paint_location['x_pos'], "y_pos":paint_location['y_pos'], "z_pos":up_height, "x_rot":pi})
        waypoint_list.append({"x_pos":paint_location['x_pos'], "y_pos":paint_location['y_pos'], "z_pos":path_height, "x_rot":pi})
        waypoint_list.append({"x_pos":paint_location['x_pos'], "y_pos":paint_location['y_pos'], "z_pos":up_height, "x_rot":pi})

        # get line
        points = contours[i]
        # add first point
        pt_canvas_x = (cx - points[0][0][0])*scale
        pt_canvas_y = (cy - points[0][0][1])*scale
        pt_scene_x = painting_canvas["x_pos"] + pt_canvas_x
        pt_scene_y = painting_canvas["y_pos"] + pt_canvas_y
        waypoint_list.append({"x_pos":pt_scene_x,"y_pos":pt_scene_y,"z_pos":up_height,"x_rot":pi})
        # lower arm at first point
        waypoint_list.append({"x_pos":pt_scene_x,"y_pos":pt_scene_y,"z_pos":path_height,"x_rot":pi})

        # add the rest of the line
        for p in range(1, len(points) - 1):
            pt_canvas_x = (cx - points[p][0][0])*scale
            pt_canvas_y = (cy - points[p][0][1])*scale
            pt_scene_x = painting_canvas["x_pos"] + pt_canvas_x
            pt_scene_y = painting_canvas["y_pos"] + pt_canvas_y
            waypoint_list.append({"x_pos":pt_scene_x,"y_pos":pt_scene_y,"z_pos":path_height,"x_rot":pi})

        # Raise arm from canvas.
        last_point = waypoint_list[-1]
        last_point["z_pos"] = up_height
        waypoint_list.append(last_point)

        cart_traj_srv_req = {
            "service":"/execute_cart_traj",
            "waypoints": waypoint_list,
            "velocity_scale_factor": 0.25
        }
        painting_service_calls.append(cart_traj_srv_req)

    print("Created trajectory with {} waypoints.".format(len(waypoint_list)))

    # move arm to the home position.
    painting_service_calls.append({"service": "/goto_home"})

    # add pig_painting_task to the database.
    panda_client.add_task("pig_painting", painting_service_calls)

    # call execute_task service for pig_painting_task.
    res = panda_client.execute_task("pig_painting")

    #print("Execute task result: {}".format(res))