#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <fstream>

//GLOBAL VARIABLES
//GLOBAL VARIABLES
float desired_speed = 0;
float current_speed = 0;
float desired_heading = 0;
float desired_anglep2p = 0;
float current_heading = 0;
float current_angle = 0;
float current_app = 0;
float current_throttle = 0;
float desired_angle = 0;
float torque_value = 0;
float turn_rate = 0;
float serial_app = 0;
float serial_bpp = 0;
float serial_torque = 0;
float previous_speed_error = 0;
float previous_steer_error = 0;
float previous_speed_pidout = 0;
float previous_steer_pidout = 0;
//float Ts = 0.02; //Loop rate
float Ts = 0.1;  //PID loop rate
float integrator = 0;
float error_d1 = 0;
ros::Time begin;
bool time_started = false;
float start_time = 0;
bool gps_data_received = false;
bool target_data_received = false;

float psidiff = 0;

std::ofstream outFile_car;
std::ofstream outFile_gps;
std::ifstream inFile_gps;

const int I_ACCUM_SIZE = 100;
std::vector<float> i_accum_speed(I_ACCUM_SIZE,0);
std::vector<float> i_accum_steer(I_ACCUM_SIZE,0);
int i_accum_iter_speed = 0;
int i_accum_iter_steer = 0;

const double pi = 3.141592653589793;
bool go = false;
float t = 0;
float x = 0; //GPS position values
float y = 0;
float z = 0;
float qx = 0; //GPS quarternion values
float qy = 0;
float qz = 0;
float qw = 0;
float vx = 0; //GPS linear velocity values
float vy = 0;
float vz = 0;
float ang_vx = 0; //GPS angular velocity values
float ang_vy = 0;
float ang_vz = 0;
double pos_covariance [36] = {}; //GPS position covariance matrix
double twist_covariance [36] = {}; //GPS twist covariance matrix
float tar_x = 0;
float tar_y = 0;
float tar_vx = 0;
float tar_vy = 0;


//Function prototypes
void gpsCallback(const nav_msgs::Odometry);
float sat2(float in, float max, float min);
void pathController();
float pi2pi(float);

//Saturation function for +/- limit value
float sat(float in, float limit){
	if(in > limit)
		return limit;
	else if(in < -limit)
		return -limit;
	else
		return in;
}

//Callback for GPS data, store in global variables to be used in controllers
void gpsCallback(const nav_msgs::Odometry msg){
	//std::cout << "In gpsCallback" << std::endl;

	if(!time_started){
		//start_time = t;
		start_time = msg.header.stamp.sec + msg.header.stamp.nsec * pow(10,-9);
		time_started = 1;
	}

	//Time difference from start of GPS node
	t = (msg.header.stamp.sec + msg.header.stamp.nsec * pow(10,-9)) - start_time;
	
	//GPS position (x,y,z)
	x = (float) msg.pose.pose.position.x;
	y = (float) msg.pose.pose.position.y;
	z = (float) msg.pose.pose.position.z;
  
	//GPS quaternion (x,y,z,w)
	qx = (float) msg.pose.pose.orientation.x;
	qy = (float) msg.pose.pose.orientation.y;  
	qz = (float) msg.pose.pose.orientation.z;
	qw = (float) msg.pose.pose.orientation.w;

	//GPS linear velocity (x,y,z)
  	vx = (float) msg.twist.twist.linear.x;
	vy = (float) msg.twist.twist.linear.y;
	vz = (float) msg.twist.twist.linear.z;

	//GPS angular velocity (x,y,z)
  	ang_vx = (float) msg.twist.twist.angular.x;
	ang_vy = (float) msg.twist.twist.angular.y;
	ang_vz = (float) msg.twist.twist.angular.z;

	//GPS covariance matrices
	//pos_covariance =  msg.pose.covariance;
	//twist_covariance = msg.twist.covariance;

	//Calculate heading
	current_heading = atan2(vy,vx);

	gps_data_received = true;
}

//Callback for virtual target
void targetCallback(const nav_msgs::Odometry msg){
	tar_x = msg.pose.pose.position.x;
	tar_y = msg.pose.pose.position.y;

	tar_vx = msg.twist.twist.linear.x;
	tar_vy = msg.twist.twist.linear.y;

	target_data_received = true;
}

//User input Callback
void usrCallback(const std_msgs::Bool in){
	if(in.data == true)
		go = true;
}

int main(int argc, char **argv){
	float Ts = 0.1; //10 Hz
 	ros::init(argc, argv, "path_planner");

  	ros::NodeHandle n;
  	begin = ros::Time::now();
  	ros::Subscriber sub_gps = n.subscribe("/gps/rtkfix", 1000, gpsCallback);
	ros::Subscriber sub_target_gps = n.subscribe("target_gps_data", 1000, targetCallback);


  	ros::Rate loop_rate(1/Ts);
  	while (ros::ok()){
	
		if(go)
			pathController();
		else{
			
			std::cout << "Waiting for Go" << std::endl;
		}
		ros::spinOnce();
    	loop_rate.sleep();
  	}
	
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

void pathController(){
	float pn = y;
	float pe = x;
	float pnt = 0;
	float pet = 0;
	float vxt = 0;
	float vyt = 0;
	float in_array [7] = {};
	int pos = 0;
	float u1 = 0;
	float u2 = 0;
	//float k1 = 0.0316; // Q1 R1000
	float k2 = 0;
	float k3 = 0;
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
	float k1 = 0.069; //Q1 R210
	float k4 = 0.069; //Q1 R210
	float vc = 0;
	float psic = 0;
	float psi = 0;
	float psidot = 0;
	float psidotp2p = 0;
	//float max_error = 0.5236;
	//float steering_angle_limit = 1000;
	//float kp = steering_angle_limit/max_error;
	float kp = 6250;

 
	/*
	std::string input;
	std::string delimiter = ",";
	std::string token;


	getline(inFile_gps,input);
	std::cout << input << std::endl;

	char* newtoken = new char[input.length()+1];

	for(int i = 0; i < 7; i++){
		pos = input.find(delimiter);
		token = input.substr(0,pos);
		std::strcpy(newtoken,token.c_str());
		in_array[i] = atof(newtoken);
		input.erase(0,pos+delimiter.length());
	}
	

	
	pet = in_array[1];
	pnt = in_array[2];
	vxt = in_array[4];
	vyt = in_array[5];
	*/

	pet = tar_x;
	pnt = tar_y;
	vxt = tar_vx;
	vyt = tar_vy;
	psi = atan2(vy,vx);
			
	//u1 = vxt - k2*(pe - pet) - k1*(pn - pnt);
	//u2 = vyt - k4*(pe - pet) - k3*(pn - pnt);
		
	u1 = vyt - k1*(pn-pnt) - k2*(pe-pet);
	u2 = vxt - k3*(pn-pnt) - k4*(pe-pet);

	vc = 2.23694*sqrt(pow(u1,2) + pow(u2,2));
	psic = atan2(u1,u2);
		
	psidot = -1*kp*pi2pi(psic - psi);

	psidiff = pi2pi(psic - psi);
	
	psidot = sat(psidot,5000);

	vc = sat2(vc,20,0);

	desired_angle = psidot;
	desired_speed = vc;
	desired_heading = psic;
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

