#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "pcan/CANMsg.h"

//------------------------GLOBAL VARIABLES------------------------//
float current_velocity = 0;
float current_app = 0;
float current_angle = 0;
float current_throttle = 0;
float desired_velocity = 0;
float app_error_d1 = 0;
float bpp_error_d1 = 0;
float appIntegrator = 0;
float bppIntegrator = 0;
int Ts = 50; //Loop rate (Hz)

//------------------------GLOBAL VARIABLES------------------------//
float appController(float,bool);
float bppController(float,bool);
float sat(float in, float limit);
float sat2(float in, float max, float min);

//Callback for CAN messages, store vehicle speed and steering wheel angle
void CANCallback(const pcan::CANMsg::ConstPtr& can_data){
	current_app = can_data->app;	
	current_velocity = can_data->mph;
	current_angle = can_data->steering_angle;
	current_throttle = can_data->throttle;
	
	if(current_velocity < 0)
		current_velocity = 0;

}

//Callback for desired speed value from high level controller
void  velocityCallback(const std_msgs::Float32 desired){
	desired_velocity = desired.data;
}


int main(int argc, char **argv){
	float appVal = 0;
	float bppVal = 0;
	float error = 0;

 	ros::init(argc, argv, "longitudinal_controller");
  	ros::NodeHandle n;
  	ros::Publisher pub_commands = n.advertise<std_msgs::Float32MultiArray>("longitudinal_commands", 1000);
	ros::Subscriber sub_CAN = n.subscribe("can_data", 1000, CANCallback);
	ros::Subscriber sub_desired = n.subscribe("desired_velocity", 1000, velocityCallback);
  	ros::Rate loop_rate(Ts);
	std::vector<float> desired (2,0);
	std_msgs::Float32MultiArray longitudinal_commands;
  	int count = 0;
	int dummy = 0;
	bool print_gps = true;
	bool print_target = true;
	bool print_once = true;
	bool call_init = true;
  
  	while (ros::ok()){


    error = desired_velocity - current_velocity;

	if(error < 0){
		appVal = 0;
		app_error_d1 = 0;
		appIntegrator = 0;
		bppVal = bppController(error,false);
	}
	else{
		if(desired_velocity == 0 && current_velocity == 0){
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

	
	desired[0] = appVal;
	desired[1] = bppVal;
	longitudinal_commands.data = desired;
	pub_commands.publish(longitudinal_commands);	
	ros::spinOnce();
    loop_rate.sleep();
  	}

  	ros::spin();
 	return 0;
}


float bppController(float error, bool reset){
	float decel_time = 2; //4 seconds to get to desired speed
	float Kp_bpp = 0.2;
	float Ki_bpp = 1.2;
	float brake_pid_limit = 2.5; //2.5 m/s^2 is the limit of passenger comfort
	float decel = 0;
	float bpp = 0;
	float u = 0;
	float u_unsat = 0;



	if(reset == 1){ //Flag can be used to reset or initalize integrator and error_d1
		bppIntegrator = 0;
		bpp_error_d1 = 0;
	}
	
	//Calculate deceleration rate (in m/s^2)
	decel = (error*0.44704)/decel_time;

	bppIntegrator = bppIntegrator + (1/(float(Ts)*2))*(decel + bpp_error_d1);
	bpp_error_d1 = decel;
	
	u = sat(Kp_bpp*decel + Ki_bpp*bppIntegrator, brake_pid_limit); //Check output saturation limits
	
	if(Ki_bpp != 0){ //Anti-windup for integral term
		u_unsat = Kp_bpp*decel + Ki_bpp*bppIntegrator;
		bppIntegrator = bppIntegrator + (1/(float(Ts)*Ki_bpp))*(u-u_unsat);
	}

	
	bpp = -0.9502*pow(u,7) - 14.14*pow(u,6) - 85.83*pow(u,5) - 273.4*pow(u,4) -490*pow(u,3) - 492.6*pow(u,2) - 268.7*u - 37.54*u;
	bpp = sat2(bpp,30,0);

	return bpp*0.01;
	
}

float appController(float error,bool reset){
	//float Kp = 1;
	//float Ki = 4.5;,
	//float Kp_app = 55; //tau = 0.25
	//float Ki_app = 112;
 
	float Kp_app = 27; //tau = 0.5
	float Ki_app = 28;
	float app = 0;
	float max_app = 20;
	float min_app = 0.0;
	float ITerm = 0;
	float limit = 60;
	float u = 0;
	float u_unsat = 0;
	
	if(reset == 1){ //Flag can be used to reset or initalize integrator and error_d1
		appIntegrator = 0;
		app_error_d1 = 0;
	}
	
	appIntegrator = appIntegrator + (1/(float(Ts)*2))*(error + app_error_d1);
	app_error_d1 = error;
	
	u = sat(Kp_app*error + Ki_app*appIntegrator, limit); //Check output saturation limits
	
	if(Ki_app != 0){ //Anti-windup for integral term
		u_unsat = Kp_app*error + Ki_app*appIntegrator;
		appIntegrator = appIntegrator + (1/(float(Ts)*Ki_app))*(u-u_unsat);
	}


	app = (u + 9.7)/3.65;
	app = sat2(app,20,0);

	return app*0.01;
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

//Saturation function for max,min limit value
float sat2(float in, float max, float min){
	if(in > max)
		return max;
	else if(in < min)
		return min;
	else
		return in;
}

