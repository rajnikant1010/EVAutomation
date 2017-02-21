#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
//#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <fstream>
#include <vector>


//------------------------GLOBAL VARIABLES------------------------//
float pe = 0; //GPS position values
float pn = 0;
float ve = 0; //GPS linear velocity values
float vn = 0;
float pet = 0;
float pnt = 0;
float vet = 0;
float vnt = 0;
bool target_data_received = false;
bool time_started = false;
std::ofstream outFile_gps;
bool gps_data_received = false;
const double pi = 3.141592653589793;


//--------------------------FUNCTIONS--------------------------//
float sat(float in, float limit);
float sat2(float in, float max, float min);
float pi2pi(float);




//Callback for GPS data, store in global variables to be used in controllers
void gpsCallback(const nav_msgs::Odometry msg){
	float t = 0;
	float z = 0;
	float qx = 0; //GPS quarternion values
	float qy = 0;
	float qz = 0;
	float qw = 0;
	float vz = 0;
	float ang_vx = 0; //GPS angular velocity values
	float ang_vy = 0;
	float ang_vz = 0;
	float current_heading = 0;
	float start_time = 0;

	if(!time_started){
		start_time = msg.header.stamp.sec + msg.header.stamp.nsec * pow(10,-9);
		time_started = 1;
	}

	//Time difference from start of GPS node
	t = (msg.header.stamp.sec + msg.header.stamp.nsec * pow(10,-9)) - start_time;
	
	//GPS position (x,y,z)
	pe = (float) msg.pose.pose.position.x;
	pn = (float) msg.pose.pose.position.y;
	z = (float) msg.pose.pose.position.z;
  
	//GPS quaternion (x,y,z,w)
	qx = (float) msg.pose.pose.orientation.x;
	qy = (float) msg.pose.pose.orientation.y;  
	qz = (float) msg.pose.pose.orientation.z;
	qw = (float) msg.pose.pose.orientation.w;

	//GPS linear velocity (x,y,z)
  	ve = (float) msg.twist.twist.linear.x;
	vn = (float) msg.twist.twist.linear.y;
	vz = (float) msg.twist.twist.linear.z;

	//GPS angular velocity (x,y,z)
  	ang_vx = (float) msg.twist.twist.angular.x;
	ang_vy = (float) msg.twist.twist.angular.y;
	ang_vz = (float) msg.twist.twist.angular.z;

	//Calculate heading
	current_heading = atan2(vn,ve);

	//Output gps data to file
	outFile_gps << t << "," << pe << "," << pn << "," << z << "," << ve << "," << vn << "," << current_heading << std::endl;

	gps_data_received = true;
}

//Callback for target gps data
void targetCallback(const nav_msgs::Odometry msg){
	pet = msg.pose.pose.position.x;
	pnt = msg.pose.pose.position.y;

	vet = msg.twist.twist.linear.x;
	vnt = msg.twist.twist.linear.y;

	target_data_received = true;
}

int main(int argc, char **argv){
	float vxt = 0;
	float vyt = 0;
	float u1 = 0;
	float u2 = 0;
	float k2 = 0;
	float k3 = 0;
	float k1 = 0.0577; //Q1 R300
	float k4 = 0.0577; //Q1 R300 
	float vc = 0;
	float psic = 0;
	float psi = 0;
	float psidot = 0;
	float psidotp2p = 0;
	float desired_angle_1 = 0;
	float desired_speed = 0;
	float desired_heading = 0;
	float psidiff = 0;
	float kp = 15000;
	int Ts = 10; //Loop rate (Hz)

	//std::vector<float> desired (2,0);


	ros::init(argc, argv, "diff_flat_state_feedback");
	ros::NodeHandle n;
	ros::Rate loop_rate(Ts);
	outFile_gps.open("/home/acostley/Desktop/steer/data_gps_1.txt");
  	//ros::Publisher pub_commands = n.advertise<std_msgs::Float32MultiArray>("vehicle_commands", 1000);
	ros::Publisher pub_velocity = n.advertise<std_msgs::Float32>("desired_velocity", 1000);
 	ros::Publisher pub_angle = n.advertise<std_msgs::Float32>("desired_angle", 1000);
  	ros::Subscriber sub_gps = n.subscribe("/gps/rtkfix", 1000, gpsCallback);
	ros::Subscriber sub_target_gps = n.subscribe("target_gps_data", 1000, targetCallback);

	//std_msgs::Float32MultiArray vehicle_commands;
	std_msgs::Float32 desired_velocity;
	std_msgs::Float32 desired_angle;

  	while (ros::ok()){

		psi = atan2(vn,ve);
		
		u1 = vnt - k1*(pn-pnt) - k2*(pe-pet);
		u2 = vet - k3*(pn-pnt) - k4*(pe-pet);

		vc = 2.23694*sqrt(pow(u1,2) + pow(u2,2));
		psic = atan2(u1,u2);
		
		psidot = -1*kp*pi2pi(psic - psi);

		psidiff = pi2pi(psic - psi);
	
		psidot = sat(psidot,5000);

		vc = sat2(vc,20,0);

		desired_angle.data = psidot;
		desired_velocity.data = vc;
		desired_heading = psic;

		//desired[0] = desired_speed;
		//desired[1] = desired_angle;
		//vehicle_commands.data = desired;
		//pub_commands.publish(vehicle_commands);
		pub_velocity.publish(desired_velocity);
		pub_angle.publish(desired_angle);

		ros::spinOnce();
    	loop_rate.sleep();
  	}

	outFile_gps.close();
  	ros::spin();

	return 0;
}


float pi2pi(float in){
	float out = in;
	if(in < -pi)
		out = in + 2*pi;
	else if(in > pi)
		out = in - 2*pi;
	return out;
}


//Saturation function for max,min limit value
float sat2(float in, float max, float min){
	if(in > max)
		return max;
	else if(in < min)
		return min;
	else
		return in;
}

//Saturation function for +/- limit value
float sat(float in, float limit){
	if(in > limit)
		return limit;
	else if(in < -limit)
		return -limit;
	else
		return in;
}
	//float max_error = 0.5236;
	//float steering_angle_limit = 1000;
	//float kp = steering_angle_limit/max_error;
	
	//float kp = 6250;
	//float kp = 18000;

	//float k1 = 0.1; // Q1 R100
	//float k4 = 0.1; // Q1 R100 
	//float k1 = 0.1414; // Q1 R50
	//float k4 = 0.1414; // Q1 R50
	//float k1 = 0.1118; // Q1 R80
	//float k4 = 0.1118; // Q1 R80
	//float k1 = 0.0913; // Q1 R120
	//float k4 = 0.0913; // Q1 R120
	//float k1 = 0.0707; // Q1 R200
	//float k4 = 0.0707; // Q1 R200
	//float k1 = 0.0816; // Q1 R150
	//float k4 = 0.0816; // Q1 R150
	//float k4 = 0.0316; // Q1 R1000
	//float k1 = 0.0845; //Q1 R140
	//float k4 = 0.0845; //Q1 R140
	//float k1 = 0.0877; //Q1 R130
	//float k4 = 0.0877; //Q1 R130
	//float k1 = 0.0791; //Q1 R160
	//float k4 = 0.0791; //Q1 R160
	//float k1 = 0.0767; //Q1 R170
	//float k4 = 0.0767; //Q1 R170
	//float k1 = 0.0674; //Q1 R220
	//float k4 = 0.0674; //Q1 R220
	//float k1 = 0.069; //Q1 R210
	//float k4 = 0.069; //Q1 R210
	//float k1 = 0.0316; // Q1 R1000


