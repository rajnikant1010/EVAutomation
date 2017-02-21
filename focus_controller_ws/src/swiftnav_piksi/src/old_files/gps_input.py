#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
import math as math
from swiftnav_piksi.msg import loc


def gps_input():
	pub = rospy.Publisher('gps_data', loc, queue_size=10)
	rospy.init_node('gps_input', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	N = 360
	N1 = 200
	i = 0

	#READ GPS DATA FROM FILE
	gps_data = sio.loadmat('/home/acostley/Desktop/kalman/data.mat')
	data = gps_data['data'];
	t = data[N1:N1+N,0]
	x = data[N1:N1+N,1]
	y = data[N1:N1+N,2]
	z = data[N1:N1+N,3]
	
	output = loc()	


	while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
		if i < N:
			output.t = t[i]
			output.x = x[i]
			output.y = y[i]
			output.z = z[i]
			rospy.loginfo(output)
			pub.publish(output)
			i = i + 1
		else:
			rospy.loginfo("End of Data")

		rate.sleep()

if __name__ == '__main__':
    try:
        gps_input()
    except rospy.ROSInterruptException:
        pass
