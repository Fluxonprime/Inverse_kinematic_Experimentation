#!/usr/bin/env python3

from __future__ import division
import rospy
import os
from pynput.keyboard import Listener, Key
from geometry_msgs.msg import Pose

def on_press(event, pb, msg):
        msg.position.x = 0
        msg.position.y = 0
        msg.position.z = 0
        if( event == Key.left):
            msg.position.x = -1
        if( event == Key.right):
            msg.position.x = 1
        if( event == Key.up):
            msg.position.y = 1
        if ( event == Key.down):
            msg.position.y = -1
        if ( event == Key.page_up):
            msg.position.z = 1
        if( event ==  Key.page_down):
            msg.position.z = -1
                            
        pb.publish(msg)
        
        
    

if __name__ == "__main__":
    rospy.init_node('input_node', anonymous=True)
    pub = rospy.Publisher('/endeff_input', Pose, queue_size=10)
    pose = Pose()
    with Listener(on_press=lambda event: on_press(event, pb=pub,msg=pose))as listener:
        listener.join()







