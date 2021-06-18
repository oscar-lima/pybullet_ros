#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_base_client():
    client= actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move x meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.y = 1.0
    
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   




if __name__ == '__main__':

    try: 
        rospy.init_node('move_base_client_v1')
        result=move_base_client()
        if result:
            rospy.loginfo("Goal Reached")

        else:
            rospy.loginfo("Failed to reach goal")

        

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

