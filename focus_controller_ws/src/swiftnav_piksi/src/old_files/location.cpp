#include "ros/ros.h"
#include <swiftnav_piksi/loc.h>
#include <swiftnav_piksi/piksi_driver.h>
#include <iostream>
#include <math.h>
#include <fstream>

swiftnav_piksi::loc submsg;
std::ofstream outFile;
bool time_started = 0;
double start_time = 0;

void positionCallback(const nav_msgs::Odometry msg)
{
  if(!time_started){
    start_time = msg.header.stamp.sec + msg.header.stamp.nsec * pow(10,-9);
    time_started = 1;
  }
  float seconds;
  std::cout << "ROVER 1" << std::endl;
  ROS_INFO("[%f]", (double) msg.pose.pose.position.x);
  ROS_INFO("[%f]", (double) msg.pose.pose.position.y);
  ROS_INFO("[%f]", (double) msg.pose.pose.position.z);
  std::cout << std::endl;

  seconds = (msg.header.stamp.sec + msg.header.stamp.nsec * pow(10,-9)) - start_time;
  submsg.t = (float) seconds;  
  submsg.x = (float) msg.pose.pose.position.x;
  submsg.y = (float) msg.pose.pose.position.y;
  submsg.z = (float) msg.pose.pose.position.z;
  
  

  outFile << seconds << "," << (float) msg.pose.pose.position.x << "," << (float) msg.pose.pose.position.y << "," << (float) msg.pose.pose.position.z << std::endl;
}


int main(int argc, char **argv)
{
  outFile.open("/home/chasekunz/Desktop/gps_data.txt");
  ros::init(argc, argv, "location");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("gps/rtkfix", 1, positionCallback);
  
  ros::Publisher pub = n.advertise<swiftnav_piksi::loc>("location",1000);//try 1 if want is needed 
  while(ros::ok())
    {
      pub.publish(submsg);
      ros::spinOnce();
    }
  ros::spin();
  outFile.close();
  return 0;
}
