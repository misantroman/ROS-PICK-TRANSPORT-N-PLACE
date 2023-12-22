#! /usr/bin/env python

import sys
import copy
import numpy as np
import tf.transformations as tran
import tf2_ros
import rospy
import actionlib
import moveit_commander
from tf2_geometry_msgs import do_transform_pose
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from tiago_iaslab_simulation.msg import Move_ArmAction, Move_ArmResult, Move_ArmFeedback, Tag_Spawn
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Empty

###################################

# objects ID's
blue_object = 1
green_object = 2
red_object = 3


pick_mode = 1

###################################

def abort(msg):
    rospy.logerr(msg)
    pick_server.set_aborted(Move_ArmResult(msg))



def succeed(msg):
    rospy.loginfo(msg)
    pick_server.set_succeeded(Move_ArmResult(msg))



def feedback(msg):
    rospy.loginfo(msg)
    pick_server.publish_feedback(Move_ArmFeedback(msg))

###################################

# add offset in Z_axis to simplify picking 
def add_z(pose, delta):
    new_pose = copy.deepcopy(pose)
    new_pose.position.z += delta
    return new_pose

# Help to transfrom a pose from one pose to another  
def pose_transform(target_frame, start_frame, pose ):
	
    transform_ok = False
    
    while not transform_ok and not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform(target_frame, start_frame, rospy.Time(0))
            map_pose = do_transform_pose(pose, transform)
            transform_ok = True
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn("Exception on transforming point... trying again \n(" + str(e) + ")")
            rospy.sleep(0.01)
    
    return map_pose

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
        abort("Action server not available!")
        return
    
    return torso_client.get_result(), torso_client.get_state()



def move_arm( pose):
    # set goal pose
    move_group_arm.set_pose_target(pose)
    # execute motion to the pose
    plan = move_group_arm.go(wait=True)
    move_group_arm.stop()
    move_group_arm.clear_pose_targets()



def safe_pose():
    # Define a safe pose for arm to allow smooth movement of the mobile base of Tiago
    safe = rospy.get_param('/pose_safe_move')
    pose_safe_move = Pose(Point(safe[0], safe[1], safe[2]), 
                     Quaternion(safe[3], safe[4], safe[5], safe[6]))
    
    feedback(" moving the arm to "'safe pose'" ")
    move_arm(pose_safe_move)
    feedback("Arm reached to safe pose and ready to move the robot mobile base")
    rospy.sleep(0.5)



def safe_pick():
    # Define a safe pose for picking the object
    s_pick = rospy.get_param('/pose_safe_pick')
    pose_safe_pick = Pose(Point(s_pick[0], s_pick[1], s_pick[2]), 
                    Quaternion(s_pick[3], s_pick[4], s_pick[5], s_pick[6]))
                    
    feedback("moving the arm to "'safe pick pose'"")
    move_arm(pose_safe_pick)
    feedback("Arm reached to "'safe pick pose'"")
    rospy.sleep(0.5)



def move_arm_to_goal(obj_pose):
    eef_orientation = tran.quaternion_from_euler(0, np.pi / 2, 0, 'rxyz')
    object_orientation = [obj_pose.orientation.x, obj_pose.orientation.y,
                        obj_pose.orientation.z, obj_pose.orientation.w]
    to_object = tran.quaternion_multiply(object_orientation, eef_orientation)
    quat_object = Quaternion(to_object[0], to_object[1], to_object[2], to_object[3])
    pose_goal = Pose(obj_pose.position, quat_object)
    feedback(" End effector pose to pick the object computed")
    move_arm(add_z(pose_goal, 0.2))
    feedback(" End effector pose to pick the object reached")



# z with respect to the frame of moveit "base_footprint"
def linear_movement_z(delta):
    move_arm(add_z(move_group_arm.get_current_pose().pose, delta))



def approach_obj():
    feedback(" Start approaching the object")
    linear_movement_z(-0.7)
    feedback(" Approached Object successfully")



def depart_obj():
    feedback(" Departing from object")
    linear_movement_z(0.07)
    feedback(" Successfully departed from object")



def move_gripper(left, right):
	
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

def attach_obj_to_gazebo(obj_id):

    # object link names in gazebo
    gazebo_links = {red_object: "cube_link",
                    blue_object: "Hexagon_link",
                    green_object: "Triangle_link"}

    # object model names in gazebo
    gazebo_models = {red_object: "cube",
                    blue_object: "Hexagon",
                    green_object: "Triangle"}

    feedback("Start attaching object to Gazebo")

    req = AttachRequest()
    req.model_name_1 = "tiago"
    req.link_name_1 = "arm_7_link"
    req.model_name_2 = gazebo_models[obj_id]
    req.link_name_2 = gazebo_links[obj_id]

    gazebo_attach.wait_for_service()
    gazebo_attach.call(req)

    feedback(" Object attached Successfully to Gazebo")
    rospy.sleep(0.1)

###################################

def generate_scene(obj_id):
    start_scene_gen.publish(Tag_Spawn(pick_mode, obj_id))
    rospy.sleep(5)
    stop_scene_gen.publish(Empty()) 

###################################

def pick_pipeline(obj_id):

    # links names
    links_tag = {blue_object: "tag_1",
                green_object: "tag_2",
                red_object: "tag_3"}

    touch_links = ["gripper_left_finger_link", "gripper_right_finger_link", "gripper_grasping_frame"]

    result, status = move_torso()
    if not status == actionlib.GoalStatus.SUCCEEDED:
        abort("Failed to lift the robot torso")
        return
    feedback("Robot torso lifted successfully")

    # spawn object on peak position to avoid wrongly read frames from apriltag
    generate_scene(obj_id)
    safe_pick()
    
    #extract the goal object
    name = links_tag[obj_id]
    dict_ret = scene.get_objects([name])
    collision_object = dict_ret[name]
    obj_pose = collision_object.primitive_poses[0]
    obj_shape = collision_object.primitives[0].dimensions
    obj_pose_stamped = PoseStamped(pose=copy.deepcopy(obj_pose))
    obj_pose_stamped.header.frame_id = "base_footprint"
    
    # pick goal object
    move_arm_to_goal(obj_pose)
    open_gripper()
    approach_obj()
    scene.remove_world_object(name)
    obj_wrt_eef = pose_transform("arm_7_link", "base_footprint", obj_pose_stamped)
    attach_obj_to_gazebo(obj_id)
    close_gripper(0.07)
    depart_obj()
    
    # add the object again to scene  with updated pose
    collision_object.primitive_poses[0] = pose_transform("base_footprint", "arm_7_link", obj_wrt_eef).pose
    scene.add_object(collision_object)
    rospy.sleep(0.1)
    
    # attach the object to grasping frame with fingers that can touch it
    move_group_arm.attach_object(name, "gripper_grasping_frame", touch_links)
    
    # go to safe pick pose then safe move base 
    safe_pick()
    safe_pose()
    
    # remove the scene except the picked object 
    # remove the attached object from known_objects and get_objects and put in get_attached_objects
    for obj in scene.get_known_object_names():
        scene.remove_world_object(obj)

###################################

# Main Pick callback that perform the whole picking task
def pick_cb(req):
	pick_pipeline(req.goal_id)
	rospy.sleep(0.1)
	pick_server.set_succeeded(Move_ArmResult("Succeeded"))

###################################

if __name__ == '__main__':
	
    rospy.init_node('pick_server')
    
    moveit_commander.roscpp_initialize(sys.argv)
    
    # gazebo attach/detach servers
    gazebo_attach = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    
    # tf for frame transformations
    tfBuffer = tf2_ros.Buffer()
    tf_l = tf2_ros.TransformListener(tfBuffer)
    
    # moving parts
    torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    robot = moveit_commander.RobotCommander()
    move_group_arm = moveit_commander.MoveGroupCommander("arm")
    
    # scene related
    scene = moveit_commander.PlanningSceneInterface()
    start_scene_gen = rospy.Publisher("generate_collision_objects", Tag_Spawn, queue_size=10)
    stop_scene_gen = rospy.Publisher("stop_spawn", Empty, queue_size=10)
    rospy.sleep(0.1)
    
    # pick_place server
    pick_server = actionlib.SimpleActionServer('pick_server', Move_ArmAction, execute_cb=pick_cb, auto_start=False)
    pick_server.start()
    rospy.loginfo("Starting Pick server ")
    rospy.spin()


