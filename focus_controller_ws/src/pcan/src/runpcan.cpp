// set here current release for this program
#define CURRENT_RELEASE "Release_20150611_n"

//****************************************************************************
// INCLUDE

#include <stdio.h>
//#include <stdlib.h>
#include <errno.h>
#include <unistd.h>   // exit
#include <signal.h>
//#include <string.h>
#include <stdlib.h>   // strtoul
#include <fcntl.h>    // O_RDWR
#include <libpcan.h>
//#include <ctype.h>
#include <stdint.h>
#include "ros/ros.h"
#include "pcan/CANMsg.h"

#include <sstream>

#include "common.h"

//****************************************************************************
// DEFINES

#define DEFAULT_NODE "/dev/pcanusb32"
#ifndef bool
#define bool	int
#define true	1
#define false	0
#endif

//****************************************************************************
// GLOBALS

HANDLE h;
const char *current_release;
__u32 rx_msg_count = 0;

//****************************************************************************
// CODE

// what has to be done at program exit
void do_exit(int error)
{
	if (h) {
		print_diag("receivetest");
		CAN_Close(h);
	}
	printf("receivetest: finished (%d): %u message(s) received\n\n",
			error, rx_msg_count);
	exit(error);
}

// the signal handler for manual break Ctrl-C
void signal_handler(int signal)
{
	do_exit(0); 
}

// what has to be done at program start
void init()
{
	/* install signal handlers */
	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);
}

void throwError(){
	do_exit(errno);
	//return errno;
}

int main(int argc, char *argv[]){

	__u16 wBTR0BTR1 = 0x1c; // baudrate 500 code 
	const char  *szDevNode = DEFAULT_NODE; // /dev/pcanusb32
	int nExtended = CAN_INIT_TYPE_ST; // standard CAN frames
	char txt[VERSIONSTRING_LEN];
	errno = 0;
	int count = 0;
	bool angular_sign = 0;

	init();
	printf("receivetest: device node=\"%s\"\n", szDevNode);
	if (nExtended == CAN_INIT_TYPE_EX)
		printf("             Extended frames are accepted");
	else
		printf("             Only standard frames are accepted");
	if (wBTR0BTR1)
		printf(", init with BTR0BTR1=0x%04x\n", wBTR0BTR1);
	else
		printf(", init with 500 kbit/sec.\n");

	//ROS initializations//
	ros::init(argc, argv, "can_publisher");
	ros::NodeHandle n;
	ros::Publisher can_pub = n.advertise<pcan::CANMsg>("can_data",1000);
	ros::Rate loop_rate(100);

	// open CAN port
	h = LINUX_CAN_Open(szDevNode, O_RDWR);
	if (!h) {
			printf("receivetest: can't open %s\n", szDevNode);
			//goto error;
			throwError();
		}

	// get version info
	errno = CAN_VersionInfo(h, txt);
	if (!errno)
		printf("receivetest: driver version = %s\n", txt);
	else {
		perror("receivetest: CAN_VersionInfo()");
		//goto error;
		throwError();
	}

	// init to a user defined bit rate
	if (wBTR0BTR1) {
		errno = CAN_Init(h, wBTR0BTR1, nExtended);
		if (errno) {
			perror("receivetest: CAN_Init()");
			//goto error;
			throwError();
		}
	}


	// Read CAN bus
	//errno = read_loop();
	// declare varialbes
	__u32 l;
	uint16_t b1,b2,b3,b4,b5,b6,b7,b8;
	uint app;
	float speed;
	pcan::CANMsg can_msg;


	// read in endless loop until Ctrl-C
	while (ros::ok()) {
		
		TPCANRdMsg m;
		__u32 status;

		if (LINUX_CAN_Read(h, &m)) {
			perror("receivetest: LINUX_CAN_Read()");
			//goto error;
			throwError();
		}

		rx_msg_count++;

		// Store important messages
		// Punlish all message after steering wheel angle is received (100Hz)
		switch(m.Msg.ID)
		{
			// Steering wheel angle
			case 0x10:
				b7 = m.Msg.DATA[6];
				b8 = m.Msg.DATA[7];
				b5 = m.Msg.DATA[4];
				can_msg.steering_angle = ((b7 << 8) + b8) - 32768;
				angular_sign = ((b5 & 128) >> 7);
				if(angular_sign == false){
					can_msg.steering_angle = can_msg.steering_angle * -1;
				}
				can_pub.publish(can_msg);
				break;
			// Speed
			case 0x75:
				b7 = m.Msg.DATA[6];
				b8 = m.Msg.DATA[7];
				speed = (float)((b7 << 8) + b8);
				can_msg.mph = (speed - 45268)/54.0f;
				break;
			// APP
			case 0x204:
				b1 = m.Msg.DATA[0];
				b2 = m.Msg.DATA[1];
				can_msg.app = ((b1 - 0xc0) << 8) + b2;
				break;
			// Throttle
			case 0x11a:
				b3 = m.Msg.DATA[2];
				b4 = m.Msg.DATA[3];
				can_msg.throttle = (b3 << 8) + b4;
				break;
			// BPP
			case 0x7D:
				b1 = m.Msg.DATA[0];
				b2 = m.Msg.DATA[1];
				can_msg.bpp = (b1 << 8) + b2;
			default:
				break;
		}

		// check if a CAN status is pending
		if (m.Msg.MSGTYPE & MSGTYPE_STATUS) {
			status = CAN_Status(h);
			if ((int)status < 0) {
				errno = nGetLastError();
				perror("receivetest: CAN_Status()");
				//goto error;
				throwError();
			}

			printf("receivetest: pending CAN status 0x%04x read.\n",
				(__u16)status);
		}

		//ros::spinOnce();
		//loop_rate.sleep();
		++count;

	}

error:
	do_exit(errno);
	return errno;
}
