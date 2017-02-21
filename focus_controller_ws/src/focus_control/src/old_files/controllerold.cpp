#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "focus_serial/serialMsg.h"
#include "focus_control/input.h"
#include "focus_control/status.h"
#include "pcan/CANMsg.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include <numeric>
#include "math.h"
#include <cmath>
#include <sstream>
#include <fstream>
#include "string.h"
#include "swiftnav_piksi/loc.h"
//using namespace std;

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
const double pi = 3.141592653589793;
bool gps_data_received = false;
bool target_data_received = false;
float psidiff = 0;
float appVal = 0;
float bppVal = 0;
float bppIntegrator = 0;
float bpp_error_d1 = 0;
float appIntegrator = 0;
float app_error_d1 = 0;

std::ofstream outFile_car;
std::ofstream outFile_gps;
//std::ifstream gps_inFile;

const int I_ACCUM_SIZE = 100;
std::vector<float> i_accum_speed(I_ACCUM_SIZE,0);
std::vector<float> i_accum_steer(I_ACCUM_SIZE,0);
int i_accum_iter_speed = 0;
int i_accum_iter_steer = 0;
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
void print();
void gpsCallback(const nav_msgs::Odometry);
void mphCallback(const pcan::CANMsg::ConstPtr&);
void inputCallback(const focus_control::input::ConstPtr&);
void longitudinalPID();
float sat(float in, float limit);
float sat2(float in, float max, float min);
float discrete_lateralPID(float y_c, float y, bool flag, float kp, float ki, float limit, float Ts);
float lateralPID();
void pathController();
float pi2pi(float);
float appController(float,bool);
float bppController(float,bool);


//Callback for GPS data, store in global variables to be used in controllers
void gpsCallback(const nav_msgs::Odometry msg){
	//std::cout << "In gpsCallback" << std::endl;

	if(!time_started){
		//start_time = t;
		start_time = msg.header.stamp.sec + msg.header.stamp.nsec * pow(10,-9);
		time_started = 1;
	}
	//t = msg.t;
	//x = msg.x;
	//y = msg.y;
	//z = msg.z;
	//vx = msg.vx;
	//vy = msg.vy;

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


	//outFile_gps << t << "," << x << "," << y << "," << z << "," << qx << "," << qy << "," << qz << "," << qw << "," << vx << "," << vy << "," << vz << "," << ang_vx << "," << ang_vy << "," << ang_vz <<  std::endl;
	outFile_gps << t << "," << x << "," << y << "," << z << "," << vx << "," << vy << "," << current_heading << std::endl;

	gps_data_received = true;
}

//Callback for CAN messages, store vehicle speed and steering wheel angle
void targetCallback(const nav_msgs::Odometry msg){
	tar_x = msg.pose.pose.position.x;
	tar_y = msg.pose.pose.position.y;

	tar_vx = msg.twist.twist.linear.x;
	tar_vy = msg.twist.twist.linear.y;

	target_data_received = true;
}

//Callback for user input,, input desired speed and heading
void inputCallback(const focus_control::input::ConstPtr& input_msg){
 	desired_speed = input_msg->desired_speed;
	desired_heading = input_msg->desired_heading;
	//desired_angle = input_msg->desired_heading;
}


int main(int argc, char **argv){

	outFile_car.open("/home/acostley/Desktop/steer/data_car_1.txt");
	outFile_gps.open("/home/acostley/Desktop/steer/data_gps_1.txt");

	std_msgs::Float32 speed_data;
	std_msgs::Float32 heading_data;
    focus_serial::serialMsg serial_data;
	geometry_msgs::PointStamped target_data;
	geometry_msgs::PointStamped location_data;
	ros::Time header_time;

		
 	ros::init(argc, argv, "controller");

  	ros::NodeHandle n;
  	begin = ros::Time::now();

  	ros::Publisher pub_serial = n.advertise<focus_serial::serialMsg>("serial_data", 1000);
  	ros::Publisher pub_speed = n.advertise<std_msgs::Float32>("car_speed", 1000);
  	ros::Publisher pub_heading = n.advertise<std_msgs::Float32>("car_desired_heading", 1000);
  	ros::Publisher pub_target = n.advertise<geometry_msgs::PointStamped>("target_location", 1000);
  	ros::Publisher pub_location = n.advertise<geometry_msgs::PointStamped>("current_location", 1000);
  	ros::Subscriber sub_can = n.subscribe("can_data",1000,mphCallback);
  	ros::Subscriber sub_input = n.subscribe("input_data",1000,inputCallback);
  	ros::Subscriber sub_gps = n.subscribe("/gps/rtkfix", 1000, gpsCallback);
	ros::Subscriber sub_target_gps = n.subscribe("target_gps_data", 1000, targetCallback);

  	ros::Rate loop_rate(1/Ts);
  


  	int count = 0;
	int dummy = 0;
	bool print_gps = true;
	bool print_target = true;
	bool print_once = true;
	bool call_init = true;
  
  	while (ros::ok()){

		if(!gps_data_received || !target_data_received){
			if(print_gps == true){
				ROS_INFO("No GPS data received");
				print_gps = false;
			}
			if(print_target == true){
				ROS_INFO("No Target data received");
				print_target = false;
			}
		}	
		else{
			if(print_once == true){
				ROS_INFO("GPS data received");
				ROS_INFO("Target data received");
				print_once = false;
			}
			//if(call_init == true){
			//	pathController();
			//	call_init = false;
			//}
		

			ROS_INFO("Desired Speed:     %f",desired_speed);
			ROS_INFO("Current Speed:     %f",current_speed);

			ROS_INFO("Desired Heading:   %f",desired_heading);
			ROS_INFO("Current Steering Angle:	%f",current_angle);
			ROS_INFO("Desired Steering Angle:	%f",desired_angle);
			
			ROS_INFO("Currnet Heading:   %f",current_heading);
			ROS_INFO("Desired Heading:	%f",desired_heading);
			ROS_INFO("Heading Diff:		%f",psidiff);

			ROS_INFO("Desired Turn Rate: %f",turn_rate);
    	

			speed_data.data = current_speed;
			heading_data.data = current_heading;
			

			//Controller Outputs
			//if(count >= (1/Ts)/10){
				//pathController(); //UNCOMMENT THIS LINE!!!
				//count = 1;
			//}
			//count++;
			
			longitudinalPID();

    		serial_data.app = appVal;
			serial_data.bpp = bppVal;
    		//serial_data.app = 0;
			//serial_data.bpp = 0;
			serial_data.steering_torque = lateralPID();
			//serial_data.steering_torque = 0.5; //comment this line out

			serial_app = serial_data.app;
			serial_bpp = serial_data.bpp;
			serial_torque = serial_data.steering_torque;

			//ROS_INFO("Commanded APP: %f",serial_data.app);
			//ROS_INFO("Commanded Torque: %f",serial_data.steering_torque);
			//ROS_INFO("Commanded BPP: %f",bppVal);
    		pub_serial.publish(serial_data);
			pub_speed.publish(speed_data); 
			pub_heading.publish(heading_data);


			//Publish current locations
			location_data.point.x = x;
			location_data.point.y = y;
			location_data.point.z = 0;
		
	
			pub_location.publish(location_data);
						

			outFile_car << ros::Time::now() - begin << ',' << x << ',' << y << ',' << vx << ',' << vy << ',' << current_heading << ',' << tar_x << ',' << tar_y << ',' << tar_vx << ',' << tar_vy << ',' << desired_heading << ',' << desired_angle << ',' << desired_speed << ',' << current_app << ',' << current_speed << ',' << current_angle << ',' <<  serial_app << ',' << serial_bpp << ',' << serial_torque << ',' << psidiff << std::endl;

			std::cout << std::endl;
			}
	ros::spinOnce();

    loop_rate.sleep();
    ++count;
  	}

	//inFile_gps.close();
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

//Print data for car states and commands
void print(){
	//outFile_car << ros::Time::now() - begin << ',' << desired_speed << ',' << current_speed << ',' << turn_rate << ',' << current_angle << ',' << torque_value << ',' << app_value << std::endl;

	outFile_car << ros::Time::now() - begin << ',' << x << ',' << y << ',' << vx << ',' << vy << ',' << current_heading << ',' << tar_x << ',' << tar_y << ',' << tar_vx << ',' << tar_vy << ',' << desired_heading << ',' << desired_angle << ',' << desired_speed << ',' << current_app << ',' << current_speed << ',' << current_angle << ',' <<  serial_app << ',' << serial_bpp << ',' << serial_torque << std::endl;
}


//Callback for CAN messages, store vehicle speed and steering wheel angle
void mphCallback(const pcan::CANMsg::ConstPtr& can_data){
	current_app = can_data->app;	
	current_speed = can_data->mph;
	current_angle = can_data->steering_angle;
	current_throttle = can_data->throttle;
	
	if(current_speed < 0)
		current_speed = 0;

}


float bppController(float error, bool reset){
	float decel_time = 2; //4 seconds to get to desired speed
	float Kp_bpp = 0.2;
	float Ki_bpp = 1.2;
	float brake_pid_limit = 2.5; //2.5 m/s^2 is the limit of passenger comfort
	float decel = 0;
	float bpp = 0;
	float pid_out = 0;
	float u = 0;
	float u_unsat = 0;



	if(reset == 1){ //Flag can be used to reset or initalize integrator and error_d1
		bppIntegrator = 0;
		bpp_error_d1 = 0;
	}
	
	//Calculate deceleration rate (in m/s^2)
	decel = (error*0.44704)/decel_time;

	bppIntegrator = bppIntegrator + (Ts/2)*(decel + bpp_error_d1);
	bpp_error_d1 = decel;
	
	u = sat(Kp_bpp*decel + Ki_bpp*bppIntegrator, brake_pid_limit); //Check output saturation limits
	
	if(Ki_bpp != 0){ //Anti-windup for integral term
		u_unsat = Kp_bpp*decel + Ki_bpp*bppIntegrator;
		bppIntegrator = bppIntegrator + (Ts/Ki_bpp)*(u-u_unsat);
	}

	pid_out = u;


	//pid_out = discrete_lateralPID(decel,0,reset,Kp_bpp,Ki_bpp,brake_pid_limit,Ts);
	
	bpp = -0.9502*pow(pid_out,7) - 14.14*pow(pid_out,6) - 85.83*pow(pid_out,5) - 273.4*pow(pid_out,4) -490*pow(pid_out,3) - 492.6*pow(pid_out,2) - 268.7*pid_out - 37.54*pid_out;

	bpp = sat2(bpp,30,0);

	return bpp*0.01;
	
}

float appController(float error,bool reset){
	//float Kp = 1;
	//float Ki = 4.5;, 
	float Kp_app = 27; //tau = 0.5
	float Ki_app = 28;
	//float Kp_app = 55; //tau = 0.25
	//float Ki_app = 112;
	float app = 0;
	float max_app = 20;
	float min_app = 0.0;
	float ITerm = 0;
	float pid_out = 0;
	//bool flag = false;
	float limit = 60;
	float u = 0;
	float u_unsat = 0;
/*
    i_accum_iter_speed++;

    if (i_accum_iter_speed > I_ACCUM_SIZE - 1)
      i_accum_iter_speed = 0;

    i_accum_speed[i_accum_iter_speed] = error;
    ITerm = (std::accumulate(i_accum_speed.begin(),i_accum_speed.end(),0))*Ki/I_ACCUM_SIZE;

    //pid_out = Kp*error + ITerm; //Comment out for discrete model
	*/
	
	//pid_out = discrete_lateralPID(desired_speed, current_speed, reset, Kp, Ki, limit, Ts); //Comment this line for "Continuous" version

    //app = 3.65*pid_out - 9.7;
	
	
	
	if(reset == 1){ //Flag can be used to reset or initalize integrator and error_d1
		appIntegrator = 0;
		app_error_d1 = 0;
	}
	
	appIntegrator = appIntegrator + (Ts/2)*(error + app_error_d1);
	app_error_d1 = error;
	
	u = sat(Kp_app*error + Ki_app*appIntegrator, limit); //Check output saturation limits
	
	if(Ki_app != 0){ //Anti-windup for integral term
		u_unsat = Kp_app*error + Ki_app*appIntegrator;
		appIntegrator = appIntegrator + (Ts/Ki_app)*(u-u_unsat);
	}

	pid_out = u;


	app = (pid_out + 9.7)/3.65;
	
	app = sat2(app,20,0);

   /* if(app > max_app)
		app = max_app;
	else if(app < min_app)
		app = min_app;
*/
	return app*0.01;
}

//Longitudinal PID Controller
void longitudinalPID(){
	float error = 0;

    error = desired_speed - current_speed;

	if(error < 0){
		appVal = 0;
		app_error_d1 = 0;
		appIntegrator = 0;
		bppVal = bppController(error,false);
	}
	else{
		if(desired_speed == 0 && current_speed == 0){
			appVal = 0;
			bppVal = bppVal;
		}
		else{
			bpp_error_d1 = 0;
			bppIntegrator = 0;
			bppVal = 0;
			appVal = appController(error,false);
		}
	}

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

//Lateral discrete PID function from "Small Unmanned Aircraft" by Randal Beard
float discrete_lateralPID(float y_c, float y, bool flag, float kp, float ki, float limit, float Ts){
	float error = 0;
	float u = 0;
	float u_unsat = 0;
	
	
	if(flag == 1){ //Flag can be used to reset or initalize integrator and error_d1
		integrator = 0;
		error_d1 = 0;
	}
	
	error = y_c - y; //Calculate current error
	integrator = integrator + (Ts/2)*(error + error_d1);
	error_d1 = error;
	
	u = sat(kp*error + ki*integrator, limit); //Check output saturation limits
	
	if(ki != 0){ //Anti-windup for integral term
		u_unsat = kp*error + ki*integrator;
		integrator = integrator + (Ts/ki)*(u-u_unsat);
	}

	return u;
}

//Lateral PID controller
float lateralPID(){
	//float Kp = 0.2;
	//float Ki = 1.8;
	float Kp = 0.3;
	float Ki = 0.5;
	float torque = 0;
	float max_torque = 63;
	float min_torque = 100 - max_torque;
	float ITerm = 0;
	float pid_out = 0;
	float error = 0;
	bool neg = false;
	float a = 0;
	float b = 0;
	float c = 0;
	float tor = 0;
	float T = 0;
	float sol1 = 0;
	float sol2 = 0;
	float discriminant = 0;
	//float desired_angle = turn_rate;
	float limit = 7000; //Limit is optimized for 15 MPH
	bool flag = false;
	float bandval = 5;
	float pos_bandval = 50+bandval;
	float neg_bandval = 50-bandval;
	



	if(error < 0){
		error = error * -1;
		neg = true;
	}

	//"Continuous" Controller
    //error = desired_angle - current_angle;
    //i_accum_iter_steer++;

   // if (i_accum_iter_steer > I_ACCUM_SIZE - 1)
    //  i_accum_iter_steer = 0;

    //i_accum_steer[i_accum_iter_steer] = error;
    //ITerm = (std::accumulate(i_accum_steer.begin(),i_accum_steer.end(),0))*Ki/I_ACCUM_SIZE;

    //pid_out = Kp*error + ITerm;	//Comment this line for Discrete version
	

	//DISCRETE MODEL
	//my derivation
	//pid_out = previous_steer_pidout + (kp+(ki*Ts/2))*error + ((ki*Ts/2)-kp)*previous_steer_error;
	//previous_steer_pidout = pid_out;
	//previous_steer_error = error;

	//Dr. Beard derivation
	pid_out = discrete_lateralPID(desired_angle, current_angle, flag, Kp, Ki, limit, Ts); //Comment this line for "Continuous" version


	if(current_speed <= 17.5 && current_speed > 12.5){
		T = (pid_out + 2120.6)/42.4125; //Uncomment if remove the 57% floor
		//T = (pid_out + 19323)/339;
		a = 128.663265306158;
		b = -14555.7517006847;
		c = 411749.632653198 - pid_out;
	}
	else if(current_speed <= 22.5 && current_speed > 17.5){
		T = (pid_out + 1816.9)/36.3375; //Uncomment if remove the 57% floor
		//T = (pid_out + 16530)/290;
		a = 75.8444183620283;
		b = -8585.83320292176;
		c = 243128.395670335 - pid_out;
	}
	else if(current_speed > 22.5){
		T = (pid_out + 1456.2)/29.125; //Uncomment if remove the 57% floor
		//T = (pid_out + 13281)/233;
		a = 59.3656755346935;
		b = -6802.71726656282;
		c = 195084.479916551 - pid_out; 
	}
	else {
		T = (pid_out + 3296.2)/65.925; //Uncomment if remove the 57% floor
		//T = (pid_out + 30039)/527;
		a = 153.303819444404;
		b = -16821.7170138839;
		c = 460472.934027625 - pid_out;
	}


	if(T < 58){
		tor = T;
	}
	else{
		discriminant = pow(b,2) - 4*a*c;
		sol1 = (-b + sqrt(discriminant))/(2*a);
		sol2 = (-b - sqrt(discriminant))/(2*a);
		if(sol1 > 58){
			tor = sol1;
		}
		else if(sol2 > 58){
			tor = sol2;
		}
		else {
			tor = 50;
		}
	}
	
	//tor = sat2(tor,63,50);

	if(tor > max_torque)
		tor = max_torque;
	else if(tor < min_torque)
		tor = min_torque;
	
	
	if(neg == false){
		torque = tor;
	}
	else{
		torque = 100 - tor;
		neg = false;
	}


	//Deadband Compensation Code
	if(torque>50){
		torque = pos_bandval + ((tor - 50)/(max_torque - 50))*(max_torque - pos_bandval);
	}
	else if(torque<50){
		torque = neg_bandval + ((50 - tor)/(50 - min_torque))*(min_torque - neg_bandval);
	}
	else {
		torque = 50;
	}


	torque_value = torque*0.01;

	return torque*0.01;
}



