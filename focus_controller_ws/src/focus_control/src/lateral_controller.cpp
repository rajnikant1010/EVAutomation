#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "pcan/CANMsg.h"

//------------------------GLOBAL VARIABLES------------------------//
float desired_angle = 0;
int Ts = 50; //Loop rate (Hz)
float current_angle = 0;
float current_speed = 0;

//-----------------------FUNCTIONS------------------------//
float sat(float in, float limit);


//------------------------CALLBACKS------------------------//
//Callback for desired speed value from high level controller
void angleCallback(const std_msgs::Float32 desired){
	desired_angle = desired.data;
}

//Callback for CAN messages, store vehicle speed and steering wheel angle
void CANCallback(const pcan::CANMsg::ConstPtr& can_data){

	current_speed = can_data->mph;
	current_angle = can_data->steering_angle;
	
	if(current_speed < 0)
		current_speed = 0;

}

int main(int argc, char **argv){
	float Kp = 0.3;
	float Ki = 0.5;
	float torque = 0;
	float max_torque = 63;
	float min_torque = 100 - max_torque;
	float ITerm = 0;
	float pid_out = 0;
	float error = 0;
	float error_d1;
	bool neg = false;
	float a = 0;
	float b = 0;
	float c = 0;
	float tor = 0;
	float T = 0;
	float sol1 = 0;
	float sol2 = 0;
	float discriminant = 0;
	float limit = 7000; //Limit is optimized for 15 MPH
	bool flag = false;
	float bandval = 5;
	float pos_bandval = 50+bandval;
	float neg_bandval = 50-bandval;
	float integrator = 0;
	float u = 0;
	float u_unsat = 0;
	float torque_value = 0;


 	ros::init(argc, argv, "lateral_controller");
  	ros::NodeHandle n;
  	ros::Publisher pub_commands = n.advertise<std_msgs::Float32>("lateral_command", 1000);
	ros::Subscriber sub_CAN = n.subscribe("can_data", 1000, CANCallback);
	ros::Subscriber sub_desired = n.subscribe("desired_angle", 1000, angleCallback);
  	ros::Rate loop_rate(Ts);
	std_msgs::Float32 lateral_command;

  	while (ros::ok()){

		error = desired_angle - current_angle; //Calculate current error

		if(error < 0){
			error = error * -1;
			neg = true;
		}


		//if(flag == 1){ //Flag can be used to reset or initalize integrator and error_d1
		//	integrator = 0;
		//	error_d1 = 0;
		//}
	

		integrator = integrator + (1/(float(Ts)*2))*(error + error_d1);
		error_d1 = error;
	
		u = sat(Kp*error + Ki*integrator, limit); //Check output saturation limits
	
		if(Ki != 0){ //Anti-windup for integral term
			u_unsat = Kp*error + Ki*integrator;
			integrator = integrator + (1/(float(Ts)*Ki))*(u-u_unsat);
		}



		if(current_speed <= 17.5 && current_speed > 12.5){
			T = (u + 2120.6)/42.4125; //Uncomment if remove the 57% floor
			//T = (pid_out + 19323)/339;
			a = 128.663265306158;
			b = -14555.7517006847;
			c = 411749.632653198 - u;
		}
		else if(current_speed <= 22.5 && current_speed > 17.5){
			T = (u + 1816.9)/36.3375; //Uncomment if remove the 57% floor
			//T = (pid_out + 16530)/290;
			a = 75.8444183620283;
			b = -8585.83320292176;
			c = 243128.395670335 - u;
		}
		else if(current_speed > 22.5){
			T = (u + 1456.2)/29.125; //Uncomment if remove the 57% floor
			//T = (pid_out + 13281)/233;
			a = 59.3656755346935;
			b = -6802.71726656282;
			c = 195084.479916551 - u; 
		}
		else {
			T = (u + 3296.2)/65.925; //Uncomment if remove the 57% floor
			//T = (pid_out + 30039)/527;
			a = 153.303819444404;
			b = -16821.7170138839;
			c = 460472.934027625 - u;
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
		torque = pos_bandval + ((torque - 50)/(max_torque - 50))*(max_torque - pos_bandval);
	}
	else if(torque<50){
		torque = neg_bandval + ((50 - torque)/(50 - min_torque))*(min_torque - neg_bandval);
	}
	else {
		torque = 50;
	}
 
	torque_value = torque*0.01;

	lateral_command.data = torque_value;
	pub_commands.publish(lateral_command);	
	ros::spinOnce();
    loop_rate.sleep();
  	}

  	ros::spin();
 	return 0;
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



