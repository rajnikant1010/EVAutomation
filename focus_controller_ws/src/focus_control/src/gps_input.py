#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
import math as math
from focus_control.msg import status
from nav_msgs.msg import Odometry

x = 0
y = 0
z = 0
vx = 0
vy = 0


def callback(data):
	global x,y,z,vx,vy,vz
	x = data.pose.pose.position.x;
	y = data.pose.pose.position.y;
	z = data.pose.pose.position.z;
	
	vx = data.twist.twist.linear.x;
	vy = data.twist.twist.linear.y;
	

def gps_input():
	gps_received = False
	pub = rospy.Publisher('target_gps_data', Odometry, queue_size=10)
	rospy.Subscriber("/gps/rtkfix", Odometry, callback)
	rospy.init_node('target_gps_input', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	#READ GPS DATA FROM FILE
	gps_data = sio.loadmat('/home/acostley/Desktop/Paths/gps_data_cw_15_stitch.mat')
	#data = gps_data['data'];
	#t = data[:,0]
	#xt = data[:,1]
	#yt = data[:,2]
	#zt = data[:,3]
	#vxt = data[:,4]
	#vyt = data[:,5]
	#heading = data[:,6];


	data = gps_data['laps'];
	xt = data[:,0]
	yt = data[:,1]
	#zt = data[:,3]
	vxt = data[:,2]
	vyt = data[:,3]
	
	output = Odometry()	
	i = 0;

	headingo = math.atan2(vyt[0],vxt[0]);
	heading_len = 5;
	error = 0;
	dist = np.zeros((20,1));

	plt.figure(1);
	plt.axis([-85, 85, -120, 50])
	h1 = plt.plot(xt,yt,'0.6',lw=12);
	h2 = plt.plot(xt[0],yt[0],'sb');
	h3 = plt.plot(xt[0],yt[0],'or');
	h4 = plt.plot([xt[0], xt[0]+heading_len*math.cos(headingo)],[yt[0], yt[0]+heading_len*math.sin(headingo)],'r');
	h5 = plt.plot([xt[0], xt[0]+heading_len*math.cos(headingo)],[yt[0], yt[0]+heading_len*math.sin(headingo)],'b');
	plt.text(-25,-20,'Speed (mph):');
	hspeed = plt.text(10,-20,'0');
	#herror = plt.text(10,-50,'0');
	plt.title('Track Map');
	plt.xlabel('X (m)');
	plt.ylabel('Y (m)');
	plt.text(-5,0,'EVR',bbox={'edgecolor':'black','facecolor':'0.6','alpha':1,'pad':40});
	
	plt.ion()
	while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
		if gps_received == False:
			if i < len(xt):
				#output.t = t[i]
				
				h = h3.pop(0)
				h.remove()
				del h
				h = h2.pop(0)
				h.remove()
				del h
				h = h4.pop(0)
				h.remove()
				del h
				h = h5.pop(0)
				h.remove()
				del h
				
				hspeed.remove();
				#herror.remove();


				heading = math.atan2(vy,vx);
				headingt = math.atan2(vyt[i],vxt[i]);
				speed = float(int(10*2.23694*(vx**2+vy**2)**0.5));
				speedstr = str(speed/10);

				#Path error calculation
				if(i > 20):
					for j in range(1,20):
						pathx = xt[i-j]; 
						pathy = yt[i-j];
						dist[j] = ((x-pathx)**2+(y-pathy)**2)**0.5;

					error = str(min(dist));
								
				
				h2 = plt.plot(xt[i],yt[i],'sb');
				h3 = plt.plot(x,y,'or');
				h4 = plt.plot([x, x+heading_len*math.cos(heading)],[y, y+heading_len*math.sin(heading)],'r');
				h5 = plt.plot([xt[i], xt[i]+heading_len*math.cos(headingt)],[yt[i], yt[i]+heading_len*math.sin(headingt)],'b');
				hspeed = plt.text(10,-20,speedstr);	
				#herror = plt.text(10,-50,error);			
				plt.draw()
				#plt.pause(0.05)
				output.pose.pose.position.x = xt[i]
				output.pose.pose.position.y = yt[i]
				#output.pose.pose.position.z = zt[i]
				output.twist.twist.linear.x = vxt[i]
				output.twist.twist.linear.y = vyt[i]
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
