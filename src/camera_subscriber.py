#! /usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Int32, Float32, String, Float64MultiArray
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from live_camera.msg import camera_message
import numpy as np
import cv2
from rospy.exceptions import ROSException

def callback(data):
    numpy_array = np.reshape(np.asarray(data.list), (227, 227, 3))
    #rospy.loginfo(rospy.get_caller_id() + "I heard ", numpy_array)
    cv2.imwrite("frame.jpg", numpy_array)
    rospy.loginfo(numpy_array)
    #sub.unregister()

def listener(): 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('listener', anonymous=True) 
    # sub = rospy.Subscriber("live_camera_image", camera_message, callback, sub)
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    rospy.init_node('listener', anonymous = True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            data = rospy.wait_for_message("live_camera_image", camera_message, timeout=1)
            print(data)
        except ROSException:
            print("timeout exceeded")
            #print(data)
        rate.sleep()
if __name__ == '__main__':
    listener()

"""
wait_for_message(topic, topic_type, timeout=None)
source code 
Receive one message from topic.

This will create a new subscription to the topic, receive one message, then unsubscribe.

Parameters:
topic (str) - name of topic
topic_type (rospy.Message class) - topic type
timeout (double) - timeout time in seconds
Returns: rospy.Message
Message
Raises:
ROSException - if specified timeout is exceeded
ROSInterruptException - if shutdown interrupts wait"""