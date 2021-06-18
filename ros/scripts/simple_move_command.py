#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
import time

 
 
 
def send_val():
 
   vel_cmd=Twist()
   rospy.init_node('move_base_pybullet',anonymous=True) #if anonymous is true it will assign a random number to end of node so each new topic is unique
   pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
  
   rate=rospy.Rate(10) #10Hz

   
   start_time=time.time()
 
   vel_cmd.linear.y=3
   vel_cmd.angular.y=0
   while (time.time()-start_time)<3:
      
       pub.publish(vel_cmd)
       rate.sleep()
 
 
 
  
  
if __name__ == "__main__":
   try:
       send_val()
   except rospy.ROSInterruptException:
       pass
