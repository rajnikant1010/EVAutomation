#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
import math as math
from focus_control.msg import status
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32




def desired_input():


	rospy.init_node('desired_data', anonymous=True)
	pubv = rospy.Publisher('desired_velocity', Float32, queue_size=10)
	puba = rospy.Publisher('desired_angle', Float32, queue_size=10)
	rate = rospy.Rate(10) # 10 Hz

	#READ GPS DATA FROM FILE
	gps_data = sio.loadmat('/home/acostley/Desktop/Paths/desired_data.mat')

	desired_velocity = gps_data['veld'];
	desired_angle = gps_data['thetad'];
	desired_heading = gps_data['psid'];
	outvel = Float32()
	outang = Float32()
	#output = FloatArray()
	#out = [desired_velocity, desired_angle, desired_heading]
	i = 0
	while not rospy.is_shutdown():
		#out = [desired_velocity[i], desired_angle[i], desired_heading[i]]
		#output.data = out
		#rospy.loginfo(desired_velocity[i])
		outvel.data = desired_velocity[i]
		outang.data = desired_angle[i]
		#outvel.data = 15
		#outang.data = 2000
		pubv.publish(outvel)
		puba.publish(outang)
		i = i +1
		rate.sleep()

if __name__ == '__main__':
    try:
        desired_input()
    except rospy.ROSInterruptException:
        pass
