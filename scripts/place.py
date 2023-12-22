#! /usr/bin/env python

import rospy
import sys
import copy
import numpy as np
import tf2_ros
import actionlib
import moveit_commander
import tf.transformations as tran
from tf2_geometry_msgs import do_transform_pose
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from tiago_iaslab_simulation.msg import Move_ArmAction, Move_ArmResult, Move_ArmFeedback, Tag_Spawn
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

###################################

# objects ID's
blue_object = 1
green_object = 2
red_object = 3

place_mode = 2

###################################

def abort(msg):
    rospy.logerr(msg)
    place_server.set_aborted(Move_ArmResult(msg))

def succeed(msg):
    rospy.loginfo(msg)
    place_server.set_succeeded(Move_ArmResult(msg))

def feedback(msg):
    rospy.loginfo(msg)
    place_server.publish_feedback(Move_ArmFeedback(msg))

###################################

def add_z(pose, delta):
    new_pose = copy.deepcopy(pose)
    new_pose.position.z += delta
    return new_pose

###################################

def move_torso():
	
    torso_client.wait_for_server()
    
    feedback("Moving torso up")
    waypoint = JointTrajectoryPoint(positions=[0.34], time_from_start=rospy.Duration(2))
    trajectory = JointTrajectory(joint_names=['torso_lift_joint'], points=[waypoint])
    torso_client.send_goal(FollowJointTrajectoryGoal(trajectory=trajectory))
    feedback("Torso Goal sent")
    
    # Waits for the server to finish performing the action.
    wait = torso_client.wait_for_result()
    
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("[PlS] Action server not available!")
        rospy.signal_shutdown("[PlS] Action server not available!")
        return
    
    # Result of executing the action
    result = torso_client.get_result()
    if result and torso_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(" torso moved correctly!")
    else:
        rospy.loginfo(" error in torso movement!")



def move_arm( pose):
    move_group_arm.set_pose_target(pose)
    plan = move_group_arm.go(wait=True)
    move_group_arm.stop()
    move_group_arm.clear_pose_targets()


 # Save pose for motion in the room 
def safe_pose():
    safe = rospy.get_param('/pose_safe_move')
    pose_safe_move = Pose(Point(safe[0], safe[1], safe[2]), 
                     Quaternion(safe[3], safe[4], safe[5], safe[6]))
    feedback(" moving the arm to "'safe pose'" ")
    move_arm(pose_safe_move)
    feedback("Arm reached to safe pose and ready to move the robot mobile base")
    rospy.sleep(0.5)


# Save pose before and after placing 
def safe_place():
    safe = rospy.get_param('/pose_safe_move')
    pose_safe_place = Pose(Point(safe[0], safe[1], safe[2]), 
                     Quaternion(safe[3], safe[4], safe[5], safe[6]))
    
    feedback("moving the arm to "'safe place pose'"")
    move_arm(pose_safe_place)
    feedback("Arm reached to "'safe place pose'" to place the object")
    rospy.sleep(0.5)


# move the arm end_effector to the right pick the object with the correct pose 
def move_arm_to_goal(obj_pose):
    eef_orientation = tran.quaternion_from_euler(0, np.pi / 2, 0, 'rxyz')
    object_orientation = [obj_pose.orientation.x, obj_pose.orientation.y,
                        obj_pose.orientation.z, obj_pose.orientation.w]
    to_object = tran.quaternion_multiply(object_orientation, eef_orientation)
    to_object = Quaternion(to_object[0], to_object[1], to_object[2], to_object[3])
    pose_goal = Pose(obj_pose.position, to_object)
    feedback(" End effector pose to place the object computed")
    move_arm(add_z(pose_goal, 0.3))
    feedback(" End effector pose to place the object reached")

# move the arm end_effector to the right place the object with the correct pose 
def move_arm_place(obj_id):
    place_hexagon = rospy.get_param('/place_hexagon')
    place_triangle = rospy.get_param('/place_triangle')
    place_cube = rospy.get_param('/place_cube')
    if obj_id == 1:
        goal = place_hexagon
        name = 'hexagon'
    elif obj_id == 2:
        goal = place_triangle
        name = 'triangle'
    elif obj_id == 3:
        goal = place_cube
        name = 'cube'
    
    # Create the action client
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    # Wait for the action server to start
    client.wait_for_server()

    # Create the goal message
    goal_msg = FollowJointTrajectoryGoal()
    goal_msg.trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
    point = JointTrajectoryPoint()
    point.positions = goal
    
    point.time_from_start = rospy.Duration(5.0)
    goal_msg.trajectory.points.append(point)

    # Send the goal to the action server
    client.send_goal(goal_msg)

    # Wait for the action to complete
    print("moving to place", name)
    client.wait_for_result()
    
    
# Get close in Z_axis to place the object to avoid falling 
def approach_obj(dz):
    waypoints = []
    scale = 1
    wpose = move_group_arm.get_current_pose().pose
    wpose.position.z -= scale * dz  # Move down along z
    waypoints.append(copy.deepcopy(wpose))

    # (waypoints to follow, eef_step, jump_threshold)
    (plan, fraction) = move_group_arm.compute_cartesian_path(waypoints,0.01,0.0)
    move_group_arm.execute(plan, wait=True)
    move_group_arm.stop()
    move_group_arm.clear_pose_targets()


# Get away with end_effector in Z_axis 
def depart_obj(dz):
    waypoints = []
    scale = 1
    wpose = move_group_arm.get_current_pose().pose
    wpose.position.z += scale * dz # Move down along z
    waypoints.append(copy.deepcopy(wpose))

    # (waypoints to follow, eef_step, jump_threshold)
    (plan, fraction) = move_group_arm.compute_cartesian_path(waypoints,0.01,0.0)

    move_group_arm.execute(plan, wait=True)
    move_group_arm.stop()
    move_group_arm.clear_pose_targets()



def move_gripper( left, right):
    gripper_client.wait_for_server()
    waypoint = JointTrajectoryPoint(positions=[left, right], time_from_start=rospy.Duration(2))
    trajectory = JointTrajectory(joint_names=['gripper_left_finger_joint', 'gripper_right_finger_joint'],
                                    points=[waypoint])
    gripper_client.send_goal(FollowJointTrajectoryGoal(trajectory=trajectory))
    feedback(" Gripper Goal sent!")
    # Waits for the server to finish performing the action.
    wait = gripper_client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        (" Gripper server not available!")
        return
    # Result of executing the action
    return gripper_client.get_result(), gripper_client.get_state()



def open_gripper():
    return move_gripper(0.25, 0.25)



def close_gripper( distance):
    return move_gripper(distance/2, distance/2)

###################################

def detach_obj(obj_id):

    # object link names in gazebo
    gazebo_links = {red_object: "cube_link",
                    blue_object: "Hexagon_link",
                    green_object: "Triangle_link"}

    # object model names in gazebo
    gazebo_models = {red_object: "cube",
                    blue_object: "Hexagon",
                    green_object: "Triangle"}

    # links names
    links_tag = {blue_object: "tag_1", green_object: "tag_2", red_object: "tag_3"}

    feedback("Start attaching object to Gazebo")

    req = AttachRequest()
    req.model_name_1 = "tiago"
    req.link_name_1 = "arm_7_link"
    req.model_name_2 = gazebo_models[obj_id]
    req.link_name_2 = gazebo_links[obj_id]

    gazebo_detach.wait_for_service()
    gazebo_detach.call(req)
    rospy.sleep(0.1)
    scene.remove_attached_object(name=links_tag[obj_id], link="gripper_grasping_frame")
    feedback("[PlS] Object detached")

###################################

def generate_scene(obj_id):
    start_scene_gen.publish(Tag_Spawn(place_mode, obj_id))
    rospy.sleep(.2)

###################################
    
def place_pipeline(obj_id):

    collision_cylinders = {blue_object: "blue_cylinder",
                      green_object: "green_cylinder",
                      red_object: "red_cylinder"}

    # links names
    links_tag = {blue_object: "tag_1", green_object: "tag_2", red_object: "tag_3"}

    # generate collision objects in the scene
    generate_scene(obj_id)
    move_torso()
    dz = 0.16
    obj_ret = scene.get_attached_objects([links_tag[obj_id]])
    obj_obj = obj_ret[links_tag[obj_id]]
    obj_pose = obj_obj.object.primitive_poses[0]
    obj_shape = obj_obj.object.primitives[0].dimensions

    dict_ret_2 = scene.get_objects([collision_cylinders[obj_id]])
    cylinder = dict_ret_2[collision_cylinders[obj_id]]
    cylinder_pose = cylinder.primitive_poses[0]
    cylinder_shape = cylinder.primitives[0].dimensions

    cylinder_pose.position.z += cylinder_shape[0]/2   # because the cylinder link is in its center
    cylinder_pose.position.z += obj_shape[0]   # because about 1/2 of the object will be external to the arm

    safe_place()
    move_arm_place(obj_id)
    
    #  if there could be swinging
    rospy.sleep(1)
    approach_obj(dz)
    
    # if there could be swinging
    rospy.sleep(1)
    open_gripper()
    detach_obj(obj_id)
    rospy.sleep(0.1)
    depart_obj(dz)
    safe_pose()
    
    # remove all collision objects from the scene 
    scene.clear()

###################################

def place_cb( req):
    # helper variables
    place_pipeline(req.goal_id)
    #move_arm_place(req.goal_id)
    rospy.sleep(0.1)
    place_server.set_succeeded(Move_ArmResult("Succeeded"))

###################################

if __name__ == '__main__':
	
    rospy.init_node('place_server')
    moveit_commander.roscpp_initialize(sys.argv)
    
    # gazebo attach/detach servers
    gazebo_detach = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    gazebo_detach.wait_for_service()
    
    # tf for frame transformations
    tfBuffer = tf2_ros.Buffer()
    tf_l = tf2_ros.TransformListener(tfBuffer)
    
    # moving parts
    gripper_client = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client.wait_for_server()
    torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    robot = moveit_commander.RobotCommander()
    move_group_arm = moveit_commander.MoveGroupCommander("arm")
    
    # scene related
    scene = moveit_commander.PlanningSceneInterface()
    start_scene_gen = rospy.Publisher("generate_collision_objects", Tag_Spawn, queue_size=10)

    place_server = actionlib.SimpleActionServer('place_server', Move_ArmAction, execute_cb=place_cb, auto_start=False)
    place_server.start()
    rospy.loginfo("Starting Place server")
    rospy.spin()


