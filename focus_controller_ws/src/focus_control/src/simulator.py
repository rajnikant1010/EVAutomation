#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
import std_msgs.msg as msg
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
import math
from swiftnav_piksi.msg import loc
from swiftnav_piksi.msg import fil
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray as FloatArray
from std_msgs.msg import Float32
from pcan.msg import CANMsg


#Initialize Global Variables
speed = 15
app = 0
bpp = 0
torque = 0.5
data_received = False



#def callback(data):

def steerK():
	global speed,torque
	torque = torque*100.0
	if speed <= 17.5 and speed > 12.5:
		#rospy.loginfo("In 1");
		return 128.663265306158*torque**2 - 14555.7517006847*torque + 411749.632653198
	elif speed <= 22.5 and speed > 17.5:
		#rospy.loginfo("In 2");		
		return 75.8444183620283*torque**2 - 8585.83320292176*torque + 243128.395670335
	elif speed > 22.5:
		#rospy.loginfo("In 3");
		return 59.3656755346935*torque**2 - 6802.71726656282*torque + 195084.479916551
	else:
		#rospy.loginfo("In 4");
		return 153.303819444404*torque**2 - 16821.7170138839*torque + 460472.934027625
	
def appK(app):
	app = app*100
	return 3.65*app - 9.7

def bppK(bpp):
	bpp = bpp*100
	return	-0.001811*bpp**2 + 0.02862*bpp - 0.3768

def longCallback(data):
	global app, bpp
	app = data.data[0]
	bpp = data.data[1]

def latCallback(data):
	global torque
	torque = data.data

def angCallback(data):
	global data_received
	data_received = True


def simulator():
	#Initialize ROS Nodes
	#pub = rospy.Publisher('filter_output', fil, queue_size=10)
	pub = rospy.Publisher('sim_out', Float32 , queue_size=10)
	pubcan = rospy.Publisher('can_data', CANMsg , queue_size=10)
	rospy.init_node('simulator', anonymous=True)
	#rospy.Subscriber("/gps/rtkfix", Odometry, callback)
	rospy.Subscriber("longitudinal_commands", FloatArray, longCallback)
	rospy.Subscriber("lateral_command", Float32, latCallback)
	rospy.Subscriber("desired_angle", Float32, angCallback)
	rate = 10
	rate = rospy.Rate(rate) # 50hz
	#desired_angle = 1000
	#torque = .60*100
	#app = 0.0
	#bpp = 0.0
	#desired_angle = 0
	decel_rate = 0
	dt = 1/10
	left = False
	count = 0
	angle = 2000
	global speed, app, bpp, torque
	
	can_data = CANMsg()
	
	while not rospy.is_shutdown():
		#Wait until desired data is received
		while data_received == False:
			rospy.loginfo("waiting for data")
			rate.sleep()
		

		#steering wheel angle
		if torque < 0.5:
			torque = 1.0 - torque  
			left = True

		angle_diff = steerK();

		if left == True:
			angle_diff = angle_diff*-1
			left = False
		
		desired_angle = angle_diff
		#desired_angle = angle + angle_diff
		angle = 0.9048*angle + 0.09516*desired_angle

		#speed
		if app != 0:	
			desired_speed = appK(app)
			speed = 0.9971*speed + 0.002853*desired_speed

		elif app == 0 and bpp == 0:
			decel_rate = 0.15 #m/s^2
			speed_diff = decel_rate*dt
			speed = speed + speed_diff*2.23694 #Convert to mph from m/s
			if speed < 0:
				speed = 0

		else:
			desired_speed = bppK(bpp)
			decel_rate = 0.9355*decel_rate + 0.06449*desired_speed
			speed_diff = decel_rate*dt
			speed = speed + speed_diff*2.23694 #Convert to mph from m/s
			if speed < 0:
				speed = 0
		
		#can_data.app = app
		can_data.mph = speed
		can_data.steering_angle = np.int16(angle)
		#can_data.bpp = bpp
		pub.publish(speed)
		pubcan.publish(can_data)
		
		rate.sleep()
	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
    try:
        simulator()
    except rospy.ROSInterruptException:
        pass

