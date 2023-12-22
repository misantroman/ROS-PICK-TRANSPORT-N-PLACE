#! /usr/bin/env python

import copy
import sys
import rospy
import tf2_ros
import numpy as np
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tran
from std_msgs.msg import Empty
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.srv import GetPlanningSceneRequest, GetPlanningScene
from tf2_geometry_msgs import do_transform_pose
from tiago_iaslab_simulation.msg import Tag_Spawn

###################################

# make object size = 1.1 * actual size 
ratio = 1.1

# flag to trigger adding objects to scene 
add_objs = False
ojects_list = []

# objects ID's
blue_object = 1
green_object = 2
red_object = 3

# arm mode
pick_mode = 1
place_mode = 2

###################################

def wait_for_object_planning_scene(object_name):
    rospy.loginfo(" Waiting for object {} to appear in planning scene...".format(object_name))
    # create service object
    gps_req = GetPlanningSceneRequest()
    gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES
    part_in_scene = False
    while not rospy.is_shutdown() and not part_in_scene:
        # This call takes a while when rgbd sensor is set
        gps_resp = scene_srv.call(gps_req)
        # check if 'part' is in the scene
        for collision_obj in gps_resp.scene.world.collision_objects:
            if collision_obj.id == object_name:
                part_in_scene = True
                break
        else:
            rospy.sleep(1.0)
    rospy.loginfo(" {} is in scene!".format(object_name))

###################################

## Define Picking and Placing Tables that we know their "pose" and "geometry" 
def add_picking_table():
    # create a box that represent table during picking
    table_position = rospy.get_param('/table_position') # (x,y,z)
    print(table_position[0])
    table_dim = (1,1,.785) # (length,width,height)
    name = "table"
    object_pose = PoseStamped()
    object_pose.pose.position = Point(float(table_position[0]),float(table_position[1]),float(table_position[2]))
    object_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    object_pose.header.frame_id = "map"
    object_pose.header.stamp = rospy.Time.now()
    scene.add_box(name, object_pose, size=[table_dim[0], table_dim[1], table_dim[2]])
    wait_for_object_planning_scene(name)



def add_placing_table(object_id):
    # position of cylinderical tables (x,y,z)
    blue_cylinder_position = rospy.get_param('/blue_cylinder_position')
    green_cylinder_position = rospy.get_param('/green_cylinder_position')
    red_cylinder_position = rospy.get_param('/red_cylinder_position')
    # cylinders dimensions (radius,height)
    cylinder_dim = (.75,0.30)

    if object_id == blue_object:
        general_cylinder("blue_cylinder", "place_table_b", float(blue_cylinder_position[0]), float(blue_cylinder_position[1]),
                                    float(blue_cylinder_position[2]), float(cylinder_dim[1]), float(cylinder_dim[0]))
    elif object_id == green_object:
        general_cylinder("green_cylinder", "place_table_g", float(green_cylinder_position[0]), float(green_cylinder_position[1]),
                                    float(green_cylinder_position[2]), float(cylinder_dim[1]), float(cylinder_dim[0]))
    elif object_id == red_object:
        general_cylinder("red_cylinder", "place_table_r", float(red_cylinder_position[0]), float(red_cylinder_position[1]),
                                    float(red_cylinder_position[2]), float(cylinder_dim[1]), float(cylinder_dim[0]))



def general_cylinder( name, identifier, x, y, z, height, radius):
    pose = PoseStamped()
    pose.pose.position = Point(x, y, z)
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    scene.add_cylinder(name, pose, height=height, radius=radius)
    # check if the box is known but not attached
    wait_for_object_planning_scene(name)

###################################

# Define objects (Triangle,Cube,Hexagon) and obstacles (Hexagons) that we only know their "geometry" and will get 
# their "pose" from Apriltag detections  
  
def add_general_cube(tag,l,w,h):
    ## used to create collision object for "red object"
    edge = .05
    name = "tag_"+str(tag.id[0])
    object_pose = PoseStamped()
    object_pose.pose.position = tag.pose.pose.pose.position
    object_pose.pose.orientation = tag.pose.pose.pose.orientation
    object_pose.header.frame_id = tag.pose.header.frame_id
    object_pose = add_z(object_pose, -0.03, name)
    square_size = [l*ratio, w*ratio, h*ratio]
    scene.add_box(name, object_pose, size=square_size)
    wait_for_object_planning_scene(name)



def add_cube(tag):
    ## used to create collision object for "red object"
    edge = .05
    name = "tag_"+str(tag.id[0])
    object_pose = PoseStamped()
    object_pose.pose.position = tag.pose.pose.pose.position
    object_pose.pose.orientation = tag.pose.pose.pose.orientation
    object_pose.header.frame_id = tag.pose.header.frame_id
    object_pose = add_z(object_pose, -edge/2, name)
    square_size = [edge*ratio, edge*ratio, edge*ratio]
    scene.add_box(name, object_pose, size=square_size)
    wait_for_object_planning_scene(name)


def add_cylinder(tag):
    ## used to create collision object for "blue object" (small ) and the obstacle (big) 
    # small cylinders dimensions (radius,height)
    small_hexagon_dim = (.03,.1)
    # big cylinders dimensions (radius,height)
    big_hexagon_dim = (.05,.2)   
    identifier = tag.id[0]
    name = "tag_"+str(identifier)
    object_pose = PoseStamped()
    object_pose.pose = tag.pose.pose.pose
    object_pose.header.frame_id = tag.pose.header.frame_id
    if identifier == 1:
        object_pose = add_z(object_pose, -small_hexagon_dim[1]/2, name)
        scene.add_cylinder(name, object_pose, height=small_hexagon_dim[1]*ratio,
                            radius=small_hexagon_dim[0]*ratio)
    else:
        object_pose = add_z(object_pose, -big_hexagon_dim[1]/2, name)
        scene.add_cylinder(name, object_pose, height=big_hexagon_dim[1]*ratio, radius=big_hexagon_dim[0]*ratio)
    wait_for_object_planning_scene(name)

###################################

def pose_transform( target_frame, start_frame, pose ):
	
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



def add_z(pose, delta_z, ref_frame):
	
	# in map frame that has z pointing up
	final_pose = pose_transform( ref_frame, pose.header.frame_id, pose)
	# subtract z
	final_pose.pose.position.z += delta_z
	# back to the original frame
	final_pose = pose_transform(final_pose.header.frame_id, pose.header.frame_id, pose) #ref_frame instead of pose
	final_pose.header.stamp = tfBuffer.get_latest_common_time(pose.header.frame_id, ref_frame)
	return final_pose

###################################

def generate_collision_objects_callback(msg):

    # add pick or place table and activate add_objs flag to start adding (objects and obstacles) in "april_callback"
    global add_objs,ojects_list
    if msg.goal_id not in [blue_object, green_object, red_object]:
        rospy.logerr(" Received target {} is not implemented".format(str(msg.goal_id)))
        return
    if msg.status == pick_mode:
        rospy.sleep(0.1)
        # list of oject ID's to avoid repeatition 
        #ojects_list = []
        add_objs = True
        add_picking_table()
        rospy.loginfo(" Spawn from Apriltag started")
    elif msg.status == place_mode:
        ojects_list = []
        add_placing_table(msg.goal_id)
    else:
        rospy.logerr(" Received phase {} is not implemented".format(str(msg.status)))


# Te deactivate scene building 
def stop_spawn_callback( empty):
    global add_objs

    add_objs = False
    rospy.logwarn(" Spawn from Apriltag stopped")


# The main function that retrive IDs and build the scene 
def april_callback(tag_array):

    # add objects and obstacles to planning scene 
    global add_objs,ojects_list
    # objects and obstacles ID's that have Hexagon shape 
    prism_list = [1, 4, 5, 6, 7]

    if add_objs:
        for tag in tag_array.detections:
            if tag.id not in ojects_list:
                ojects_list.append(tag.id)
                if tag.id[0] in prism_list:
                    add_cylinder(tag)
                elif tag.id[0] == red_object:
                    add_cube(tag)
                elif tag.id[0] == green_object:
                    add_general_cube(tag,l=.03,w=.03,h=.02)
                    #add_pyramid(tag)

###################################

if __name__ == "__main__":
	
    rospy.init_node("process_tags_node")
    
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Scene clients and functions
    scene = moveit_commander.PlanningSceneInterface()
    scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    rospy.loginfo("Waiting for scene service")
    scene_srv.wait_for_service()
    rospy.loginfo(" Scene service avilable")

    ## Transform listener to allow transformation from one frame to another
    tfBuffer = tf2_ros.Buffer()
    tf_l = tf2_ros.TransformListener(tfBuffer)

    # subscribe to the topics
    sub_april = rospy.Subscriber("tag_detections", AprilTagDetectionArray, april_callback)
    start_obj_generation = rospy.Subscriber("generate_collision_objects", Tag_Spawn, generate_collision_objects_callback)
    end_obj_generation = rospy.Subscriber("stop_spawn", Empty, stop_spawn_callback)

    # spinning the callbacks
    rospy.spin()


