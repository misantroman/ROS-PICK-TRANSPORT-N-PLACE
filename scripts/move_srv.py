#! /usr/bin/env python

import rospy
import actionlib
from math import radians, degrees
from tf.transformations import quaternion_from_euler
from tiago_iaslab_simulation.msg import MoveAction, MoveResult, MoveFeedback
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

###################################

# ojects ID's
blue_object  = 1
green_object  = 2
red_object = 3

# pick/place modes
pick_status = 1
place_status = 2
    

###################################

def abort(msg):
    rospy.logerr(msg)
    action_server.set_aborted(MoveResult(msg))



def succeed(msg):
    rospy.loginfo(msg)
    action_server.set_succeeded(MoveResult(msg))



def feedback(msg):
    rospy.loginfo(msg)
    action_server.publish_feedback(MoveFeedback(msg))

###################################

#this method will make the robot move to the goal location
def go_to_goal(pose):

    #define a client for to send goal requests to the "move_base" server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")

    # Creates a new goal object 
    goal = MoveBaseGoal()

    # set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # set goal position (x,y,z) w.r.t "map" frame
    goal.target_pose.pose.position = pose.position

    # set goal orientation using "Quaternion" w.r.t "map" frame
    goal.target_pose.pose.orientation = pose.orientation

    # Sends the goal to the action server.
    ac.send_goal(goal)

    wait = ac.wait_for_result(rospy.Duration(60))
    
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        abort(" Failed to recieve answer from move_base server ")
    else:
        # Result of executing the action
        return ac.get_result(), ac.get_state()



def move_head(j1, j2):
	
        # Waits until the action server has started up and started listening for goals.
        client_head.wait_for_server()
        rospy.loginfo("Moving head server started")

        waypoint = JointTrajectoryPoint(positions=[j1, j2], time_from_start=rospy.Duration(2))
        trajectory = JointTrajectory(joint_names=['head_1_joint', 'head_2_joint'], points=[waypoint])
        goal = FollowJointTrajectoryGoal(trajectory=trajectory)

        # Sends the goal to the action server.
        client_head.send_goal(goal)

        rospy.loginfo("Sending Head Goal!")
        
        # Waits for the server to finish performing the action.
        wait = client_head.wait_for_result()
        
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            abort("Error server is not available")
            return
        # Result of executing the action
        return client_head.get_result(), client_head.get_state()



def reset_head():
    return move_head(0, -0.3)



def move_to_pick(obj_id):

    # # Get picking poses from parameter server 
    blue_object_pick = rospy.get_param('/blue_object_pick') 
    green_object_pick = rospy.get_param('/green_object_pick') 
    red_object_pick = rospy.get_param('/red_object_pick') 

    pick_poses = {blue_object: Pose(Point(blue_object_pick[0], blue_object_pick[1], blue_object_pick[2]), 
                            Quaternion(blue_object_pick[3], blue_object_pick[4], blue_object_pick[5], blue_object_pick[6])),

                green_object: Pose(Point(green_object_pick[0], green_object_pick[1], green_object_pick[2]), 
                            Quaternion(green_object_pick[3], green_object_pick[4], green_object_pick[5], green_object_pick[6])),

                red_object: Pose(Point(red_object_pick[0], red_object_pick[1], red_object_pick[2]), 
                            Quaternion(red_object_pick[3], red_object_pick[4], red_object_pick[5], red_object_pick[6]))}

    # head configuration for pick for each object
    
    head_pick_pose = {blue_object: [0.27, -0.7],
                    green_object: [-0.2, -0.7],
                    red_object: [0.4, -0.7]}    
    # Initial pick pose 
    init_pose = rospy.get_param('/robot_init_pick_pose') 
    init_pick_pose = Pose(Point(init_pose[0], init_pose[1], init_pose[2]), 
                     Quaternion(init_pose[3], init_pose[4], init_pose[5], init_pose[6]))
   
                
    # head initial position
    result, status = reset_head()
    if not status == actionlib.GoalStatus.SUCCEEDED:
        abort("Error reaching initial head pose on pick mode")
        return
    feedback(" Initial head pose on pick mode reached")

    # initial picking pose
    result, status = go_to_goal(init_pick_pose)
    if not status == actionlib.GoalStatus.SUCCEEDED:
        abort(" Error in reaching safe position on pick mode")
        return
    feedback("Intermediate position on pick mode reached")

    # final picking pose
    result, status = go_to_goal(pick_poses[obj_id])
    if not status == actionlib.GoalStatus.SUCCEEDED:
        abort(" Error reaching the final position on pick mode")
        return
    feedback(" Final position on pick mode reached")

    # move head to final pose
    result, status = move_head(head_pick_pose[obj_id][0], head_pick_pose[obj_id][1])
    if not status == actionlib.GoalStatus.SUCCEEDED:
        abort(" Error reaching the final head pose on pick mode")
        return
    feedback(" Final head pose on pick mode reached")
    
    succeed(" Motion to pick object completed")



def move_to_place(obj_id):

    # Get placing poses from parameter server
    blue_object_place = rospy.get_param('/blue_object_place') 
    green_object_place = rospy.get_param('/green_object_place') 
    red_object_place = rospy.get_param('/red_object_place') 

    place_poses = {blue_object: Pose(Point(blue_object_place[0], blue_object_place[1], blue_object_place[2]), 
                           Quaternion(blue_object_place[3], blue_object_place[4], blue_object_place[5], blue_object_place[6])),

            green_object: Pose(Point(green_object_place[0], green_object_place[1], green_object_place[2]), 
                           Quaternion(green_object_place[3], green_object_place[4], green_object_place[5], green_object_place[6])),

            red_object: Pose(Point(red_object_place[0], red_object_place[1], red_object_place[2]), 
                           Quaternion(red_object_place[3], red_object_place[4], red_object_place[5], red_object_place[6]))} 


    # head configuration for palce for each object
    head_place_pose = [0, -0.3]
    # Initial pick pose
    init_pose = rospy.get_param('/robot_init_place_pose')
    init_place_pose = Pose(Point(init_pose[0], init_pose[1], init_pose[2]), 
                     Quaternion(init_pose[3], init_pose[4], init_pose[5], init_pose[6]))    
                    
    # head standard position
    result, status = reset_head()
    if not status == actionlib.GoalStatus.SUCCEEDED:
        abort(" Error reaching standard head pose on place mode")
        return
    feedback(" Standard head pose on place mode reached")
    
    # intermediate safe pose
    result, status = go_to_goal(init_place_pose)
    if not status == actionlib.GoalStatus.SUCCEEDED:
        abort(" Error reaching intermediate position on place mode")
        return
    feedback(" Intermediate position on place mode reached")
    
    # final placing pose
    result, status = go_to_goal(place_poses[obj_id])
    if not status == actionlib.GoalStatus.SUCCEEDED:
        abort("Error reaching the final position on place mode")
        return
    feedback(" Final position on place mode reached")
    
    # head final pose
    result, status = move_head(head_place_pose[0], head_place_pose[1])
    if not status == actionlib.GoalStatus.SUCCEEDED:
        abort(" Error reaching the final head pose on place mode")
        return
    feedback(" Final head pose on place mode reached")
    succeed(" Motion to target completed")

###################################

# Main function that perform the whole motion process
def main_cb(req):
	
    obj_id = req.goal_id
    obj_status = req.status
    
    # Check valid objects only 
    if obj_id not in [blue_object, green_object, red_object]:
        abort("Target {"+str(obj_id)+"} not valid")
        return

    if obj_status == pick_status:
        move_to_pick(obj_id)
    elif obj_status == place_status:
        move_to_place(obj_id)
    else:
        abort("Phase {"+str(obj_status)+"} not implemented")

###################################

if __name__ == '__main__':
	
    rospy.init_node('move_server')
    
    client_head = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.sleep(0.1)
    action_server = actionlib.SimpleActionServer('move_server', MoveAction, execute_cb = main_cb, auto_start=False)
    action_server.start()
    
    rospy.loginfo(" Action server is now avilable!")
    
    rospy.spin()


