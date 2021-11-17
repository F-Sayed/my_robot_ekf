#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include "geometry_msgs/Twist.h"
using namespace std;

int dir;

/*
dir map:
0: fwd_right
1: fwd_left
2: fwd_straight

3: bwd_right
4: bwd_left
5: bwd_straight

6: rot_right
7: rot_left
8: still
*/

void cmd_vel_callback(const geometry_msgs::Twist msg)
{
  if (msg.linear.x > 0)
  {
    if (msg.angular.z > 0) dir = 0;
    else if (msg.angular.z < 0) dir = 1;
    else if (msg.angular.z == 0) dir = 2;   
  }
  
  if (msg.linear.x < 0)
  {
    if (msg.angular.z > 0) dir = 3;
    else if (msg.angular.z < 0) dir = 4;
    else if (msg.angular.z == 0) dir = 5;   
  }
  
  if (msg.linear.x == 0)
  {
    if (msg.angular.z > 0) dir = 6;
    else if (msg.angular.z < 0) dir = 7;
    else if (msg.angular.z == 0) dir = 8;   
  }
}


int main(int argc, char** argv){

  ros::init(argc, argv, "speed_publisher");

  ros::NodeHandle n;
  ros::Publisher speed_pub = n.advertise<std_msgs::String>("speed_values", 50);
  
  ros::Subscriber sub = n.subscribe("cmd_vel", 10, cmd_vel_callback);
  
  ros::Rate r(100);

  std_msgs::String msg;
  
  //delta of encoder ticks
  int L_SPEED = 0;
  int R_SPEED = 0;

  while (n.ok())
  {
    ros::spinOnce();               // check for incoming messages

    switch (dir)
    {
      case 0:
        L_SPEED = 150;
        R_SPEED = 120;
        break;
      case 1:
        L_SPEED = 120;
        R_SPEED = 150;
        break;
      case 2:
        L_SPEED = 150;
        R_SPEED = 150;
        break;
      case 3:
        L_SPEED = -150;
        R_SPEED = -120;
        break;
      case 4:
        L_SPEED = -120;
        R_SPEED = -150;
        break;
      case 5:
        L_SPEED = -150;
        R_SPEED = -150;
        break;
      case 6:
        L_SPEED = 100;
        R_SPEED = -100;
        break;
      case 7:
        L_SPEED = -100;
        R_SPEED = 100;
        break;
      case 8:
        L_SPEED = 0;
        R_SPEED = 0;
        break;
      default:
        L_SPEED = 0;
        R_SPEED = 0;
        break;
    }
    
    stringstream ss;
    ss << L_SPEED << " " << R_SPEED;
    
    msg.data = ss.str();
    speed_pub.publish(msg);

    r.sleep();
  }
  }
  
  
