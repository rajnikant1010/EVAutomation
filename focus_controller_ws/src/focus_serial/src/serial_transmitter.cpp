#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <bitset>
#include "serial/serial.h"
#include "ros/ros.h"
#include "focus_serial/serialMsg.h"
#include "geometry_msgs/Twist.h"

using namespace std;

uint32_t baud = 115200;
string port = "";
//string port = "/dev/ttyUSB0";
//string port = "/dev/ttyACM0";
serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
float app = 0.0;
float brake = 0.0;
float steer = 0.0;

void rearrangebytes(uint8_t *data){
	uint8_t temp = data[0];
	data[0] = data[3];
	data[3] = temp;
	temp = data[1];
	data[1] = data[2];
	data[2] = temp;
}

uint16_t byteswap_16(uint16_t data){
	uint8_t swap[2];
	uint8_t temp;
	memcpy(&swap[0], &data, sizeof(data));
	temp = swap[0];
	swap[0] = swap[1];
	swap[1] = temp;
	memcpy(&data, &swap[0], sizeof(data));
	return data;
}

void printArray(uint8_t *data, int length){
	int j;
	for (j=0;j<length; j++){
		bitset<8> b(data[j]);
		cout << b << " ";
	}
	cout << endl;
}

void printArrayHex(uint8_t *data, int length){
	int j;
	for (j=0;j<length; j++){
		printf("%2X ",data[j]);
	}
	printf("\n");
}

void printuint_16(uint16_t data){
	bitset<16> b(data);
	cout << b << endl;
}

uint16_t calculateCRC(uint8_t* data, int length){
	uint16_t crc = 0;
	int i;

	for (i=0; i < length; i++){
		crc = (uint8_t)(crc >> 8) | (crc << 8);
		crc ^= data[i];
	    crc ^= (uint8_t)(crc & 0xff) >> 4;
	    crc ^= crc << 12;
	    crc ^= (crc & 0x00ff) << 5;
	    //printuint_16(crc);
	}

	return crc;
}

void serialCallback(const focus_serial::serialMsg::ConstPtr& msg){ //normal
//void serialCallback(const geometry_msgs::Twist::ConstPtr& msg){ // teleop
	uint8_t data[16];
	uint16_t crc;
	float newAPP = app;
	float newSteer = steer;


	// Normal
	float app = msg->app;
	float brake = msg->bpp;
	float steer = msg->steering_torque;

	cout << "app: " << app << endl;
	cout << "torque: " << steer << endl;
	cout << "bpp:	" << brake << endl;
	
/*	
	// Teleop
	newAPP += (float)msg->linear.x * 0.01;
	//brake += msg->serial_brake;
	newSteer += (float)msg->angular.z * 0.01;

	if(newAPP < 0)
		app = 0.0;
	else if(newAPP > 0.99999)
		app = 0.9;
	else
		app = newAPP;

	if(newSteer < -0.99999)
		steer = -0.9;
	else if(newSteer > 0.999999)
		steer = 0.9;
	else steer = newSteer;

	cout << "app: " << app << endl;
	cout << "brake: " << brake << endl;
	cout << "steering: " << steer << endl;
	*/


	// Construct packet
	data[0] = 0xFA; // begin packet
	data[1] = 12;	// number of bytes in payload
	memcpy(&data[2], &app, sizeof(app));
	memcpy(&data[6], &brake, sizeof(brake));
	memcpy(&data[10], &steer, sizeof(steer));
	//printArrayHex(data,16);


	crc = calculateCRC(&data[2],12); // calculate CRC
	//printf("CRC before swap: %X\n", crc);
	crc = byteswap_16(crc);
	//printf("CRC after swap: %X\n", crc);

	memcpy(&data[14], &crc, sizeof(crc)); // copy CRC into array

	size_t bytes_written = my_serial.write(data,16);

	cout << bytes_written << " bytes written" << endl << endl;

}

int main(int argc, char **argv){

	// declare variables
	uint8_t data[16];
	uint8_t temp;
	uint16_t crc;

	// ROS initializations
	ros::init(argc, argv, "serial_transmitter");
	ros::NodeHandle n;
	ros::NodeHandle n_("~");
	ros::Subscriber sub = n.subscribe("serial_data", 1000, serialCallback); // normal
	//ros::Subscriber sub = n.subscribe("cmd_vel", 1000, serialCallback); //teleop
	n_.param<string>("port",port,"dev/ttyUSB7");

	my_serial.setPort(port);
	my_serial.open();

	if(my_serial.isOpen())
		ROS_INFO("Serial Port Open On: %s",port.c_str());
  	else
    	ROS_INFO("Serial Port Not Open");

	ros::spin();
	return 0;
}
