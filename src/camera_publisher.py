#! /usr/bin/python

#remove or add the library/libraries for ROS
import rospy, time, math, random
from rospy.numpy_msg import numpy_msg
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from live_camera.msg import camera_message
from std_msgs.msg import Int32, Float32, String, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
import cv2
import numpy as np
from scipy.misc import imresize

def talker(camera_serial_port):
	camera_serial_port = camera_serial_port
	pub = rospy.Publisher("live_camera_image", camera_message, queue_size=10)
	rospy.init_node("camera_publisher")

	rate = rospy.Rate(10)

	cap = cv2.VideoCapture(int(camera_serial_port[-1]))
	cap.release()

	while not rospy.is_shutdown():
		cap = cv2.VideoCapture(int(camera_serial_port[-1]))
		ret, frame = cap.read()

		frame_cropped = frame[:, 80:560, :]
		img = imresize(frame_cropped, (227, 227, 3))
		img = np.swapaxes(img, 0, 1)

		#training_mean= [123.68, 116.779, 103.939] 
		#img = img - training_mean 
		#img[:, :, 0], img[:, :, 2] = img[:, :, 2], img[:, :, 0]

		cap.release()

		#img = np.ones((2,2,3)).tolist()
		#img = np.ones((2, 1)).tolist()
		#img = [1.0, 1.0]
		data = camera_message()
		data.list = img.flatten().tolist()

		pub.publish(data)
		#rospy.loginfo(data)
		rospy.loginfo(img)
		rate.sleep()


if __name__=='__main__':
	camera_serial_port = "/dev/video1"
	try:
		talker(camera_serial_port)
	except rospy.ROSInterruptException:
		pass




"""import rospy
	 4 from std_msgs.msg import String
	 5 
	 6 def talker():
	 7     pub = rospy.Publisher('chatter', String, queue_size=10)
	 8     rospy.init_node('talker', anonymous=True)
	 9     rate = rospy.Rate(10) # 10hz
	10     while not rospy.is_shutdown():
	11         hello_str = "hello world %s" % rospy.get_time()
	12         rospy.loginfo(hello_str)
	13         pub.publish(hello_str)
	14         rate.sleep()
	15 
	16 if __name__ == '__main__':
	17     try:
	18         talker()
	19     except rospy.ROSInterruptException:
	20         pass"""


"""Listener example:

	 from rospy.numpy_msg import numpy_msg

	 rospy.init_node('mynode')
	 rospy.Subscriber("mytopic", numpy_msg(TopicType)
Publisher example:

	 from rospy.numpy_msg import numpy_msg
	 import numpy
	 
	 pub = rospy.Publisher('mytopic', numpy_msg(TopicType))
	 rospy.init_node('mynode')
	 a = numpy.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=numpy.float32)
	 pub.publish(a)"""

	 		#pub.publish(img)

"""
MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
		A = np.ones((2, 3, 4))

		dim = [MultiArrayDimension("height", A.shape[0], 1), MultiArrayDimension("width", A.shape[1], 1), MultiArrayDimension("rgb_channels", A.shape[2], 1)]
		#dim = [1.0, 1.0, 1.0]
		data_offset = 0

		layout = MultiArrayLayout(dim, data_offset) 
		
		data = Float64MultiArray(layout, A.tolist())"""