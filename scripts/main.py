#! /usr/bin/env python

import rospy
import numpy
import random
import actionlib
from tiago_iaslab_simulation.srv import Objs, ObjsRequest, ObjsResponse
from tiago_iaslab_simulation.msg import MoveAction, MoveGoal, MoveResult, MoveFeedback
from tiago_iaslab_simulation.msg import Move_ArmAction, Move_ArmResult,Move_ArmGoal, Move_ArmFeedback


pick_status = 1
place_status = 2


def get_objects_ID():

    # Get the list of objects ID's by calling human server
    rospy.loginfo("Waiting human service...")
    human_cl.wait_for_service()
    rospy.loginfo("Human service is runing ")

    # create a request 
    request = ObjsRequest()
    request.ready = True
    request.all_objs = True
    response = human_cl.call(request)

    # check if response in valid
    if response is not None:
        if len(response.ids) > 0:
            rospy.loginfo(str(response.ids)+" received from human node")
            return response.ids
    else:
        rospy.logwarn(" Failed to recieve response from human node ")
    return



def move(id , status):

    '''
        Input :
                - object Id = 1 for blue object
                            = 2 for green object
                            = 3 for red object

                - status = 1 for picking 
                         = 2 for palcing

        Output : True/False 

        Function : Move the robot to desired location based on input (id , status)
    '''
    
    # wait "navigate_server" to be avilable
    navigate_cl.wait_for_server()

    # send request based on the status = pick/place
    if status == pick_status :

        navigate_cl.send_goal(MoveGoal(pick_status, id))
    else:
        navigate_cl.send_goal(MoveGoal(place_status, id))

    wait = navigate_cl.wait_for_result()
    if not wait:
        rospy.logerr("Move server not available")
        return False
    result = navigate_cl.get_result() 

    # check if we execute the motion correctly
    if result and navigate_cl.get_state() == actionlib.GoalStatus.SUCCEEDED:
        if status == pick_status:
            rospy.loginfo("Motion to the pick position completed correctly")
        else :
            rospy.loginfo("Motion to the place position completed correctly")
    else:
        if status == place_status:
            rospy.loginfo("Failed to move to the pick position")
        else:
            rospy.loginfo("Failed to move to the place position")
        return False
    return True



def act(id , status):

    '''
        Input :
                - object Id = 1 for blue object
                            = 2 for green object
                            = 3 for red object

                - status = 1 for picking 
                         = 2 for palcing

        Output : True/False 

        Function : Call pick/place server based on input (id , status) 
    '''
    
    # send request based on the status = pick/place
    if status == pick_status :

        # wait for "pick_server" to be avilable
        pick_cl.wait_for_server()
        
        # send request of the object we want to pick        
        pick_cl.send_goal(Move_ArmGoal(id))
        wait = pick_cl.wait_for_result()
        if not wait:
            rospy.logerr("Pick server not available")
            return False
        
        result = pick_cl.get_result()
        
        # check if we execute the picking correctly
        if result and pick_cl.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Pick action completed correctly")
        else:
            rospy.loginfo("Failed to pick the object")
            return False
        
        return True
        
    else:
		
        # wait for "place_server" to be avilable
        place_cl.wait_for_server()
        
        # send request of the object we want to place
        place_cl.send_goal(Move_ArmGoal(goal_id=id))
        wait = place_cl.wait_for_result()
        if not wait:
            rospy.logerr("Pick server not available")
            return False
        
        result = place_cl.get_result()
        
        # check if we execute the placing correctly
        if result and place_cl.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Place action completed correctly")
        else:
            rospy.loginfo("Failed to place the object")
            return False
        
        return True



if __name__ == '__main__':
	
	try:
		rospy.init_node('main_node')
		
		# create human client to call human server to get objects ID's
		human_cl = rospy.ServiceProxy("human_objects_srv", Objs) 
		# create navigation client to call move_server that calls move_base service 
		navigate_cl = actionlib.SimpleActionClient('move_server', MoveAction)
		# create object pick client to perform picking task
		pick_cl = actionlib.SimpleActionClient('pick_server', Move_ArmAction)
		# create object place client to perform placing task
		place_cl = actionlib.SimpleActionClient('place_server', Move_ArmAction)

        # Get ID List from human node
		obj_id_list = get_objects_ID()

		# iterate over each ID and perform pick & place task 
		for i in range(len(obj_id_list)):
			if obj_id_list[i] is not None:
				# move to target pose to pick this object 
				if move(obj_id_list[i],pick_status):
					# After reching the picking pose perform picking action
					if act(obj_id_list[i],pick_status):
						# After picking the object , move to target pose to place this object 
						if move(obj_id_list[i],place_status):
							# After reching the placing pose perform placing action
							if act(obj_id_list[i],place_status):
								rospy.loginfo("Finished Pick an Place Task for object {}".format(i+1))
								

	except rospy.ROSInterruptException:
		rospy.loginfo("ROS Interrupt Exception!")
		
		
