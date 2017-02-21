#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
import math as math
from focus_control.msg import status
from nav_msgs.msg import Odometry



def gps_input():
	gps_received = False
	pub = rospy.Publisher('/gps/rtkfix', Odometry, queue_size=10)

	rospy.init_node('fake_gps_input', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	#READ GPS DATA FROM FILE
	gps_data = sio.loadmat('/home/acostley/Desktop/Paths/gps_data_cw_15_stitch.mat')


	data = gps_data['laps'];
	xt = data[:,0]
	yt = data[:,1]
	#zt = data[:,3]
	vxt = data[:,2]
	vyt = data[:,3]
	
	output = Odometry()	
	i = 0;

	while not rospy.is_shutdown():
		output.pose.pose.position.x = xt[i]
		output.pose.pose.position.y = yt[i]
		#output.pose.pose.position.z = zt[i]
		output.twist.twist.linear.x = vxt[i]
		output.twist.twist.linear.y = vyt[i]
		i = i + 1
		rospy.loginfo(output)
		pub.publish(output)
		rate.sleep()

if __name__ == '__main__':
    try:
        gps_input()
    except rospy.ROSInterruptException:
        pass
