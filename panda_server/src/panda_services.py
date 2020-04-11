#!/usr/bin/env python

import rospy
import actionlib
import tf
import sys

import panda_server.srv
import franka_gripper.msg
import std_srvs.srv

import moveit_commander
import moveit_commander.conversions as conversions

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Vector3, Quaternion
from moveit_msgs.msg import Constraints, RobotTrajectory

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray


class PandaServer():
    def __init__(self, group="arm"):
        # Planning_scene provides an interface to the robot's environment.
        self.planning_scene = moveit_commander.PlanningSceneInterface()
        # Publish waypoints to RVIZ so they can be visualized using tiny spheres.
        self.display_waypoints = rospy.Publisher('visualization_marker', Marker, latch=True, queue_size=1000)
        # Interface to the group of joints called "arm".
        self.commander = moveit_commander.MoveGroupCommander(group)
        # Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
        self.robot = moveit_commander.RobotCommander()
        # Get the end effector link.
        self.eef_frame = self.commander.get_end_effector_link()
        rospy.loginfo("Planning with end effector frame: {}".format(self.eef_frame))
        # Get the reference frame for cartesian planning.
        self.ref_frame = self.commander.get_planning_frame()
        rospy.loginfo("Planning with reference frame: {}".format(self.ref_frame))

        # Declare ROS services.
        rospy.Service("/set_canvas", panda_server.srv.SetCanvas, self.setCanvas)
        rospy.loginfo("Starting service '/set_canvas'")

        rospy.Service("/set_workspace", panda_server.srv.SetWorkspace, self.setWorkspace)
        rospy.loginfo("Starting service '/set_workspace'")

        rospy.Service("/goto_gripper_pos", panda_server.srv.GripperPos, self.goToGripperPos)
        rospy.loginfo("Starting service '/goto_gripper_pos'")

        rospy.Service("/grasp", panda_server.srv.Grasp, self.grasp)
        rospy.loginfo("Starting service '/grasp'")

        rospy.Service("/goto_joint_pos", panda_server.srv.JointPos, self.goToJointPos)
        rospy.loginfo("Starting service '/goto_joint_pos'")

        rospy.Service("/goto_cart_pos", panda_server.srv.CartPose, self.goToCartPos)
        rospy.loginfo("Starting service '/goto_cart_pos'")

        rospy.Service("/execute_cart_traj", panda_server.srv.CartTraj, self.executeCartTraj)
        rospy.loginfo("Starting service '/execute_cart_traj'")

        rospy.Service("/goto_home", panda_server.srv.Home, self.goToHome)
        rospy.loginfo("Starting service '/goto_home'")

        rospy.Service("/scale_limits", panda_server.srv.ScaleLimits, self.scaleLimits)
        rospy.loginfo("Starting service '/scale_limits'")

    # Converts our custom message types to the geometry_msgs/PoseStamped messages used by MoveIt.
    def msgToPoseStamped(self, msg, frame_id):
        q = tf.transformations.quaternion_from_euler(msg.x_rot, msg.y_rot, msg.z_rot)
        posestamped = PoseStamped(
            header=Header(frame_id=frame_id, stamp=rospy.Time.now()),
            pose = Pose(position=Point(msg.x_pos, msg.y_pos, msg.z_pos), orientation=Quaternion(q[0],q[1],q[2],q[3]))
        )
        return posestamped

    # Lower the velocity and acceleration limits set in the URDF by scaling them by a value 0-1.
    def scaleLimits(self, req):
        try:
            if req.velocity_scale_factor:
                self.commander.set_max_velocity_scaling_factor(req.velocity_scale_factor)
                rospy.loginfo("scaling max velocity by: {}".format(req.velocity_scale_factor))
            if req.acceleration_scale_factor:
                self.commander.set_max_acceleration_scaling_factor(req.acceleration_scale_factor)
                rospy.loginfo("scaling max acceleration by: {}".format(req.acceleration_scale_factor))
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr("Error scaling limits: {}".format(e))
            return False
        return True

    # Add a canvas to the planning scene.
    # This will be visually displayed in MoveIt and also be added as a collision object for motion planning.
    def setCanvas(self, req):
        rospy.loginfo("Setting canvas.")
        # If there is an old canvas, remove it.
        self.planning_scene.remove_world_object("canvas")
        # X,Y,Z dimensions are required to define the canvas. Check to make sure we have them.
        if not any(msg_field is None for msg_field in (req.canvas.x_dim, req.canvas.y_dim, req.canvas.z_dim)):
            canvas = self.msgToPoseStamped(req.canvas, self.ref_frame)
            try:
                # We can reference the canvas object in other fuctions by the name we set here: "canvas"
                self.planning_scene.add_box("canvas", canvas, (req.canvas.x_dim, req.canvas.y_dim, req.canvas.z_dim))
                rospy.loginfo("Adding canvas to planning scene.")
            except moveit_commander.MoveItCommanderException as e:
                rospy.logerr("Error adding canvas to the planning scene: {}".format(e))
                return False
        else:
            rospy.logerr("PaintingTask service request must contain a valid canvas message! (x_dim, y_dim, z_dim)")
            # Return appropriate bool value for the 'success' field of the service response.
            return False
        return True

    # Move arm to home position.
    def goToHome(self, req):
        rospy.loginfo("Moving Panda to home position.")
        try:
            joint_positions = [0,-0.785,0,-2.356,0,1.571,0.785]
            self.commander.go(joint_positions, wait=True)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(str(e))
            # Return appropriate bool value for the 'success' field of the service response.
            return False
        return True

    # move gripper to a new position.
    def goToGripperPos(self, req):
        client = actionlib.SimpleActionClient('move', franka_gripper.msg.MoveAction)
        rospy.loginfo("Waiting for 'move' gripper action server.")
        client.wait_for_server()
        rospy.loginfo("Recieved 'move' gripper action server.")
        goal = franka_gripper.msg.MoveGoal(width=req.width, speed=req.speed)
        client.send_goal(goal)
        rospy.loginfo("Sent goal: width = {}, speed = {} to 'move' gripper action server.".format(req.width, req.speed))
        client.wait_for_result()
        success = client.get_result()
        return success

    def grasp(self, req):
        client = actionlib.SimpleActionClient('grasp', franka_gripper.msg.GrapAction)
        client.wait_for_server()
        goal = franka_gripper.msg.GrapGoal(width=req.width, epsilon=req.epsilon, speed=req.speed, force=req.force)
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    # send a joint position command.
    def goToJointPos(self, req):
        joint_goal_list = list(req.joint_goal_list)
        try:
            # If goal is relative, add the command to the current position.
            if req.relative:
                # get current joint positions.
                current_pos = self.commander.get_current_joint_values()
                joint_goal_list = list(map(add, joint_goal_list, current_pos))
            rospy.loginfo("Executing joint space trajectory: {}".format(joint_goal_list))
            # Execute goal.
            self.commander.go(joint_goal_list, wait=True)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(str(e))
            # Return appropriate bool value for the 'success' field of the service response.
            return False
        return True

    # send a cartesian position command.
    def goToCartPos(self, req):
        waypoint_msg = self.msgToPoseStamped(req.waypoint, self.ref_frame)
        goal = waypoint_msg.pose
        try:
            rospy.loginfo("Going to cartesian position: {}".format(goal))
            # Execute goal.
            self.commander.go(goal, wait=True)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(str(e))
            # Return appropriate bool value for the 'success' field of the service response.
            return False
        return True

    def executeCartTraj(self, req):
        waypoint_list = []
        current_postamped = self.commander.get_current_pose()
        waypoint_list.append(current_postamped.pose)
        for waypoint in req.waypoints:
            waypoint_msg = self.msgToPoseStamped(waypoint, self.ref_frame)
            waypoint_list.append(waypoint_msg.pose)
        # Display waypoints in Moveit as tiny spheres.
        if req.display_waypoints:
            rospy.loginfo("Displaying trajectory waypoints.")
            visual_points =  Marker(
                type=Marker.SPHERE_LIST,
                header=Header(frame_id=self.ref_frame),
                scale=Vector3(0.01, 0.01, 0.01),
                color=ColorRGBA(0.0, 0.2, 1.0, 0.3),
                action=Marker.ADD
            )
            z_offset = 0.1
            for waypoint in waypoint_list:
                visual_points.points.append(Point(waypoint.position.x, waypoint.position.y, waypoint.position.z-z_offset))
                self.display_waypoints.publish(visual_points)
        rospy.loginfo("requested trajectory with {} waypoints.".format(str(len(waypoint_list))))
        try:
            # Specify whether the robot is allowed to replan if it detects changes in the environment.
            self.commander.allow_replanning(True)
            # Set the position tolerance that is used for reaching the goal when moving to a pose. 
            self.commander.set_goal_position_tolerance(0.2)
            # Set the orientation tolerance that is used for reaching the goal when moving to a pose.
            self.commander.set_goal_orientation_tolerance(0.2)
            # set planner.
            self.commander.set_planner_id("RTTConnectkConfigDefault")
            # set max planning time in seconds.
            self.commander.set_planning_time(100)
            # Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints.
            # Configurations are computed for every eef_step meters.
            # The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resulting path.
            # Kinematic constraints for the path given by path_constraints will be met for every point along the trajectory, if they are not met, a partial solution will be returned.
            # The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory.
            eef_step = 0.0001
            jump_threshold = 0.00
            fraction = 0.0
            attempts = 0
            while fraction < 1.0 and attempts < 100:
                (plan_msg, fraction) = self.commander.compute_cartesian_path(waypoint_list, eef_step, jump_threshold, avoid_collisions=True, path_constraints=None)
                rospy.loginfo("cartesian path {}%% acheived".format(str(fraction*100.0)))
                attempts += 1
                if attempts % 10 == 0: 
                    rospy.loginfo("Still planning trajectory after " + str(attempts) + " attempts...")
            if fraction == 1.0: 
                rospy.loginfo("Path computed successfully.") 
            else: 
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after 100 attempts.")
                return False
            # Time parameterization.
            plan_msg = self.commander.retime_trajectory(self.robot.get_current_state(), plan_msg, req.velocity_scale_factor)

            # check for other waypoints with identical durations and remove one of the points.
            valid_points = []
            prev_time = rospy.Duration.from_sec(-1) # tmp placeholder.
            for point in plan_msg.joint_trajectory.points:
                cur_time = point.time_from_start
                if cur_time == prev_time:
                    rospy.loginfo("Waypoints with equal times: {}, {}. Removing one waypoint.".format(prev_time, cur_time))
                else:
                    valid_points.append(point)
                prev_time = cur_time
            # Use the valid waypoints for path plan.
            plan_msg.joint_trajectory.points = valid_points
                
            # Execute the path plan.
            self.commander.execute(plan_msg) 
            rospy.loginfo("Path execution complete.")
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr(str(e))
            # Return appropriate bool value for the 'success' field of the service response.
            return False
        return True

    def setWorkspace(self, req):
        workspace = [req.minX, req.minY, req.minZ, req.maxX, req.maxY, req.maxZ]
        rospy.loginfo("Setting workspace [minX, minY, minZ, maxX, maxY, maxZ] -> {}".format(workspace))
        try:
            self.commander.set_workspace(workspace)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr("Error setting workspace: {}".format(e))
            return False
        return True


if __name__=="__main__":
    # Initialize the move_group API.
    moveit_commander.roscpp_initialize(sys.argv)
    # Initialize ROS node.
    rospy.init_node('panda_services', anonymous=True)
    # Initialize moveit services.
    painter = PandaServer()
    # Wait for services to be called.
    rospy.spin()
    # Shutdown the move_group API.
    moveit_commander.roscpp_shutdown()