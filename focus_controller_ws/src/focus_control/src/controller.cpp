#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "pcan/CANMsg.h"
#include <fstream>
#include "nav_msgs/Odometry.h"
#include <vector>
#include <ctime>
#include <cstring>
#include <sys/stat.h>
#include "focus_serial/serialMsg.h"

//------------------------GLOBAL VARIABLES------------------------//
std::ofstream outFile_car;
std::ofstream outFile_gps;
float t = 0;
float x = 0;
float y = 0;
float vx = 0;
float vy = 0;
float current_heading = 0;
float tar_x = 0;
float tar_y = 0;
float tar_vx = 0;
float tar_vy = 0; 
float desired_heading = 0;
float desired_angle = 0;
float desired_velocity = 0;
float current_app = 0;
float current_velocity = 0;
float current_angle = 0;
float serial_app = 0;
float serial_bpp = 0;
float serial_torque = 0;
bool time_started = false;
float start_time = 0;
float current_time = 0;
bool gps_data_received = false;
bool target_data_received = false;
bool CAN_data_received = false;
float current_throttle = 0;
bool ready = false;


//------------------------CALLBACKS------------------------//
//Callback for GPS data, store in global variables to be used in controllers
void gpsCallback(const nav_msgs::Odometry msg){
	float z = 0;
	float qx = 0; //GPS quarternion values
	float qy = 0;
	float qz = 0;
	float qw = 0;
	float vz = 0;
	float ang_vx = 0; //GPS angular velocity values
	float ang_vy = 0;
	float ang_vz = 0;

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

	//Calculate heading
	current_heading = atan2(vy,vx);

	gps_data_received = true;
}

//Callback for Virtual Target
void targetCallback(const nav_msgs::Odometry msg){
	tar_x = msg.pose.pose.position.x;
	tar_y = msg.pose.pose.position.y;

	tar_vx = msg.twist.twist.linear.x;
	tar_vy = msg.twist.twist.linear.y;

	target_data_received = true;
}


//Callback for CAN messages, store vehicle speed and steering wheel angle
void CANCallback(const pcan::CANMsg::ConstPtr& can_data){
	CAN_data_received = true;
	current_app = can_data->app;	
	current_velocity = can_data->mph;
	current_angle = can_data->steering_angle;
	current_throttle = can_data->throttle;
	
	if(current_velocity < 0)
		current_velocity = 0;
}

//Callback for desired angle from high level controller
void angleCallback(const std_msgs::Float32 desired){
	desired_angle = desired.data;
}

//Callback for desired speed value from high level controller
void velocityCallback(const std_msgs::Float32 desired){
	desired_velocity = desired.data;
}

//Callback for lateral steering command (steering torque value)
void lateralCallback(const std_msgs::Float32 torque){
	serial_torque = torque.data;
}

//Callback for longitudinal commands (app and bpp)
void longitudinalCallback(const std_msgs::Float32MultiArray command){
	serial_app = command.data[0];
	serial_bpp = command.data[1];
}

//-----------------------FUNCTION PROTOTYPES------------------------//
void print();
void name_out_file();

int main(int argc, char **argv){
	int Ts = 10;
	ros::init(argc, argv, "controller");
  	ros::NodeHandle n;
	ros::Publisher pub_serial = n.advertise<focus_serial::serialMsg>("serial_data", 1000);
	ros::Subscriber sub_CAN = n.subscribe("can_data", 1000, CANCallback);
  	ros::Subscriber sub_gps = n.subscribe("/gps/rtkfix", 1000, gpsCallback);
	ros::Subscriber sub_desired_velocity = n.subscribe("desired_velocity", 1000, velocityCallback);
	ros::Subscriber sub_desired_angle = n.subscribe("desired_angle",1000, angleCallback);
	ros::Subscriber sub_lateral = n.subscribe("lateral_command",1000,lateralCallback);
	ros::Subscriber sub_longitudinal = n.subscribe("longitudinal_commands",1000,longitudinalCallback);
	ros::Subscriber sub_target_gps = n.subscribe("target_gps_data", 1000, targetCallback);  	
	ros::Rate loop_rate(Ts);
	name_out_file();
	

	focus_serial::serialMsg serial_data;
	//outFile_car.open("/home/acostley/Desktop/steer/ignore.txt");

 
  	while (ros::ok()){
		if(CAN_data_received == false || gps_data_received == false || target_data_received == false){
			ROS_INFO("-----------------------");
			if(CAN_data_received == false)
				ROS_INFO("CAN Data:  NO!");
			else
				ROS_INFO("CAN Data:  YES!");

			if(gps_data_received == false)
				ROS_INFO("GPS Data:  NO!");
			else
				ROS_INFO("GPS Data:  YES!");

			if(target_data_received == false)
				ROS_INFO("TAR Data:  NO!");
			else
				ROS_INFO("TAR Data:  YES!");
			ROS_INFO("----------------------");
		}
		else{
			ROS_INFO("-----------------------");
			ROS_INFO("CAN Data:  YES!");
			ROS_INFO("GPS Data:  YES!");
			ROS_INFO("TAR Data:  YES!");
			ROS_INFO("----------------------");
			ready = true;
			
			current_time = ros::Time::now().toSec();
			print();
		}
		
		serial_data.app = serial_app;
		serial_data.bpp = serial_bpp;
		serial_data.steering_torque = serial_torque;
		pub_serial.publish(serial_data);
		
		if(ready == true){
			print();
		}
		ros::spinOnce();
    	loop_rate.sleep();
  	}

  	ros::spin();
 	return 0;
}




//------------------------FUNCTIONS------------------------//
//Print data for car states and commands
void print(){
			outFile_car << t << ',' << x << ',' << y << ',' << vx << ',' << vy << ',' << current_heading << ',' << tar_x << ',' << tar_y << ',' << tar_vx << ',' << tar_vy << ',' << desired_heading << ',' << desired_angle << ',' << desired_velocity << ',' << current_app << ',' << current_velocity << ',' << current_angle << ',' <<  serial_app << ',' << serial_bpp << ',' << serial_torque << std::endl;
}

void name_out_file(){
	char year[4];
	char month[2];
	char day[2];
	char hour[2];
	char min[2];
	char sec[2];
	std::string leadingz;
	std::string filename;
	std::string filepath;
	std::string folder;
	int tmp;
	time_t now = time(0);
	tm *ltm = localtime(&now);
	int status = 0;
    struct stat sb;

	//Get month, day, year, hour and min from tm struct
	tmp = sprintf(year, "%d", 1900+ltm->tm_year);
	tmp = sprintf(month, "%d", 1+ltm->tm_mon);
	tmp = sprintf(day, "%d", ltm->tm_mday);
	tmp = sprintf(hour, "%d", ltm->tm_hour);
	tmp = sprintf(min, "%d", ltm->tm_min);
	tmp = sprintf(sec, "%d", ltm->tm_sec);

	//Check to add leading 0 for month, day, hour, and min
	if(atof(month) < 10){
		leadingz = "0"+std::string(month);
		strcpy(month,leadingz.c_str());
	}
	if(atof(day) < 10){
		leadingz = "0"+std::string(day);
		strcpy(day,leadingz.c_str());
	}
	if(atof(hour) < 10){
		leadingz = "0"+std::string(hour);
		strcpy(hour,leadingz.c_str());
	}
	if(atof(min) < 10){
		leadingz = "0"+std::string(min);
		strcpy(min,leadingz.c_str());
	}
	if(atof(sec) < 10){
		leadingz = "0"+std::string(sec);
		strcpy(sec,leadingz.c_str());
	}
	//std::cout << leadingz << std::endl;
	
	//Convert char* to string for concactenation with filepath
	filename = "data_"+std::string(hour)+"_"+std::string(min)+"_"+std::string(sec)+".txt";
	folder  = "/home/acostley/Desktop/focus_out/out_files/" + std::string(month)+std::string(day)+std::string(year)+"/";


    if (stat(folder.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))
    {
        //Folder Exists
    }
    else
    {
       	status = mkdir(folder.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }


	filepath = folder + filename;
	outFile_car.open(filepath.c_str());
	//outFile_car << "Hello World" << std::endl;
}




