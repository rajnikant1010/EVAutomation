#!/usr/bin/env python
# license removed for brevity
import rospy
#from std_msgs.msg import String
import std_msgs.msg as msg
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
import math
from swiftnav_piksi.msg import loc
from swiftnav_piksi.msg import fil

#Initialize Global Variables
data_received = False
no_gps_data = True
gps_t = 0 
gps_t_last = -0.1
gps_x = 0
gps_y = 0
gps_z = 0

def callback(data):
	global gps_t, gps_x, gps_y, gps_z, data_received,no_gps_data
	gps_t = data.t
	gps_x = data.x
	gps_y = data.y
	gps_z = data.z
	data_received = True
	no_gps_data = False
	

def kalman():
	#Initialize ROS Nodes
	pub = rospy.Publisher('filter_output', fil, queue_size=10)
	rospy.init_node('kalman', anonymous=True)
	rospy.Subscriber("gps_data", loc, callback)

	#Initialize Variables
	global gps_t, gps_x, gps_y, gps_z, gps_t_last,data_received,no_gps_data
	filter_output = fil()
	#Filter Initialization
	Pfilter = np.array([(1**2,0,0,0),(0,1**2,0,0),(0,0,5**2,0),(0,0,0,5**2)])
	Q = 1*np.array([(1**2,0,0,0),(0,1**2,0,0),(0,0,2**2,0),(0,0,0,2**2)])
	R = 0.03**2*np.array([(1,0),(0,1)])
	F = np.array([(0,0,1,0),(0,0,0,1),(0,0,0,0),(0,0,0,0)])
	H = np.array([(1,0,0,0),(0,1,0,0)])
	M = 10
	pi = math.pi

	#Open Output File
	f = open('/home/acostley/Desktop/output_data','w')
	
	while no_gps_data:
		rospy.loginfo("No GPS Data Received")

	rospy.loginfo("GPS Data Received")
	#Xdata = np.array([(0),(0),(0),(0)])
	Xfilter = np.array([(gps_x),(gps_y),(0.1),(0.1)]) #At t=0
	Xdata = Xfilter



	rate = rospy.Rate(100) # 100hz (10 times faster than GPS)
	while not rospy.is_shutdown():
		#dt = gps_t - gps_t_last
		dt = 0.1
		#Prediction
		xdot = np.array([(Xfilter[2]),(Xfilter[3]),(10),(5)])
		Xfilter = Xfilter + (dt/M)*xdot
		Pfilter = Pfilter + (dt/M)*(F.dot(Pfilter)+Pfilter.dot(F.transpose())+Q)		
	
		if data_received == True:
			data_received = False
			tmp = np.linalg.inv(R+H.dot(Pfilter).dot(H.transpose()))
			K = Pfilter.dot(H.transpose()).dot(tmp)
			xy = np.array([(gps_x),(gps_y)]);
			Xfilter_xy = np.array([(Xfilter[0]),(Xfilter[1])])
			Xfilter = Xfilter - K.dot(Xfilter_xy - xy)
			Pfilter = (np.identity(4) - K.dot(H)).dot(Pfilter)
			Xdata = Xfilter

		xf = Xdata[0]
		yf = Xdata[1]
		vxf = Xdata[2]
		vyf = Xdata[3]
		vel = math.sqrt(vxf**2 + vyf**2)*2.23694
		psi = math.atan2(vyf,vxf)*180/pi
		
		filter_output.t = gps_t
		filter_output.x = gps_x
		filter_output.y = gps_y
		filter_output.xf = xf
		filter_output.yf = yf
		filter_output.vxf = vxf
		filter_output.vyf = vyf
		filter_output.vel = vel
		filter_output.psi = psi
		
		f.write(repr(gps_t)+','+repr(gps_x)+','+repr(gps_y)+','+repr(xf)+','+repr(yf)+','+repr(vxf)+','+repr(vyf)+','+repr(vel)+','+repr(psi)+'\n')
		gps_t_last = gps_t
		rospy.loginfo(filter_output)
		pub.publish(filter_output)
		rate.sleep()
	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	f.close()


if __name__ == '__main__':
    try:
        kalman()
    except rospy.ROSInterruptException:
        pass


   

    


