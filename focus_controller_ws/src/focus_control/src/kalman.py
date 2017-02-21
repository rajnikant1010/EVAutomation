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
from nav_msgs.msg import Odometry

#Initialize Global Variables
data_received = False
no_gps_data = True
gps_t = 0 
gps_t_last = -0.1
gps_x = 0
gps_y = 0
gps_z = 0
time_started = False
start_time = 0
xcov = 1000;
ycov = 1000;
vxcov = 1000;
vycov = 1000;

def callback(data):
	global gps_t, gps_x, gps_y, gps_z, data_received,no_gps_data,gps_vx,gps_vy,time_started,start_time,xcov,ycov,vxcov,vycov
	no_gps_data = False

	if time_started == False:
		start_time = data.header.stamp.secs + (data.header.stamp.nsecs*(10**(-9)));
		time_started = True
	
	gps_t = data.header.stamp.secs + (data.header.stamp.nsecs*(10**(-9))) - start_time;

	gps_x = data.pose.pose.position.x;
	gps_y = data.pose.pose.position.y;
	gps_z = data.pose.pose.position.z;
	
	gps_vx = data.twist.twist.linear.x;
	gps_vy = data.twist.twist.linear.y;
	xcov = data.pose.covariance[0];
	ycov = data.pose.covariance[7];
	vxcov = data.pose.covariance[21];
	vycov = data.pose.covariance[28];
		

	data_received = True


def kalman():
	#Initialize ROS Nodes
	#pub = rospy.Publisher('filter_output', fil, queue_size=10)
	pub = rospy.Publisher('filter_output', Odometry, queue_size=10)
	rospy.init_node('kalman', anonymous=True)
	rospy.Subscriber("/gps/rtkfix", Odometry, callback)



	#Initialize Variables
	global gps_t, gps_x, gps_y, gps_z, gps_t_last,data_received,no_gps_data,xcov,ycov,vxcov,vycov
	#filter_output = fil()
	filter_output = Odometry()
	t = rospy.Time.now()
	sec = t.secs
	nsec = t.nsecs
	current_time_s = sec + nsec*(10**-9)
	#current_time_s = rospy.get_time()
	#current_time_s = current_time.to_sec
	last_time_s = current_time_s
	vxdata_last = 0
	vydata_last = 0
 


	#Filter Initialization
	Pfilter = np.array([(1**2,0,0,0),(0,1**2,0,0),(0,0,5**2,0),(0,0,0,5**2)])
	Q = 10*np.array([(1**2,0,0,0),(0,1**2,0,0),(0,0,1**2,0),(0,0,0,1**2)])
	#R = 0.03**2*np.array([(1,0),(0,1)])
	R = np.array([(xcov,0,0,0),(0,ycov,0,0),(0,0,vxcov,0),(0,0,0,vycov)])
	F = np.array([(0,0,1,0),(0,0,0,1),(0,0,0,0),(0,0,0,0)])
	H = np.array([(1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1)])
	M = 10
	pi = math.pi

	#Open Output File
	#f = open('/home/acostley/Desktop/ece6320/corrupted_data/kalman_out_cor','w')
	rate = rospy.Rate(100) # 100hz (10 times faster than GPS)


	while no_gps_data and not rospy.is_shutdown():
		#rospy.loginfo("No GPS Data Received")
		rate.sleep()




	rospy.loginfo("GPS Data Received")
	#Xdata = np.array([(0),(0),(0),(0)])
	#Xfilter = np.array([(gps_x),(gps_y),(0.1),(0.1)]) #At t=0
	Xfilter = np.array([(gps_x),(gps_y),(gps_vx),(gps_vy)]) #At t=0
	Xdata = Xfilter
	vxdata_last = Xdata[2]
	vydata_last = Xdata[3]



	
	while not rospy.is_shutdown():
		R = np.array([(xcov,0,0,0),(0,ycov,0,0),(0,0,vxcov,0),(0,0,0,vycov)])
		#dt = gps_t - gps_t_last
		dt = 0.1
		#current_time = rospy.get_time()
		#current_time_s = current_time.to_sec
		t = rospy.Time.now()
		sec = t.secs
		nsec = t.nsecs
		current_time_s = sec + nsec*(10**-9)
		#dt = current_time_s - last_time_s
		
		ax = (Xfilter[2] - vxdata_last)/dt
		ay = (Xfilter[3] - vydata_last)/dt

		#Prediction
		#xdot = np.array([(Xfilter[2]),(Xfilter[3]),(10),(5)])
		xdot = np.array([(Xfilter[2]),(Xfilter[3]),(0),(0)])
		Xfilter = Xfilter + (dt/M)*xdot
		Pfilter = Pfilter + (dt/M)*(F.dot(Pfilter)+Pfilter.dot(F.transpose())+Q)		
	
		#Measurement
		if data_received == True:
			data_received = False
			tmp = np.linalg.inv(R+H.dot(Pfilter).dot(H.transpose()))
			K = Pfilter.dot(H.transpose()).dot(tmp)
			states = np.array([(gps_x),(gps_y),(gps_vx),(gps_vy)]);
			#Xfilter_xy = np.array([(Xfilter[0]),(Xfilter[1])])
			#Xfilter = Xfilter - K.dot(Xfilter_xy - xy)
			Xfilter = Xfilter - K.dot(Xfilter - states)
			Pfilter = (np.identity(4) - K.dot(H)).dot(Pfilter)

		Xdata = Xfilter

		xf = Xdata[0]
		yf = Xdata[1]
		vxf = Xdata[2]
		vyf = Xdata[3]
		vel = math.sqrt(vxf**2 + vyf**2)*2.23694
		psi = math.atan2(vyf,vxf)*180/pi
		
		#filter_output.t = gps_t
		#filter_output.x = gps_x
		#filter_output.y = gps_y
		#filter_output.vx = gps_vx
		#filter_output.vy = gps_vy
		#filter_output.xf = xf
		#filter_output.yf = yf
		#filter_output.vxf = vxf
		#filter_output.vyf = vyf
		#filter_output.vel = vel
		#filter_output.psi = psi

		filter_output.header.stamp = rospy.Time.from_sec(gps_t)
		filter_output.pose.pose.position.x = xf
		filter_output.pose.pose.position.y = yf
		
		filter_output.twist.twist.linear.x = vxf
		filter_output.twist.twist.linear.y = vyf
		
		#f.write(repr(gps_t)+','+repr(gps_x)+','+repr(gps_y)+','+repr(gps_vx)+','+repr(gps_vy)+','+repr(xf)+','+repr(yf)+','+repr(vxf)+','+repr(vyf)+','+repr(vel)+','+repr(psi)+','+repr(dt)+','+repr(current_time_s)+','+repr(last_time_s)+'\n')
		
		gps_t_last = gps_t
		last_time_s = current_time_s
		vxdata_last = vxf
		vydata_last = vyf

		#rospy.loginfo(filter_output)
		#rospy.loginfo(xcov);
		#rospy.loginfo(ycov);
		#rospy.loginfo(vxcov);
		#rospy.loginfo(vycov);


		pub.publish(filter_output)
		rate.sleep()
	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	#f.close()


if __name__ == '__main__':
    try:
        kalman()
    except rospy.ROSInterruptException:
        pass


   

    


