#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"

#include <sstream>
#include <string>
using namespace std;


const double DISTANCE_PER_TICK = 0.000143;
const float CHASIS_LENGTH = 0.17;   
int left_encoder = 0;
int right_encoder = 0;

int left_encoder_prev = 0;
int right_encoder_prev = 0;

geometry_msgs::Quaternion quat;

/*
void cmd_vel_callback(const geometry_msgs::Twist msg)
{
  ROS_INFO_STREAM(msg.linear.x);
  
  if(msg.linear.x > 0)
    vx = 0.5;
    
  else if(msg.linear.x < 0)
    vx = -0.5;
   
  if(msg.linear.x == 0)
    vx = 0;
    
  if(msg.angular.z > 0)
    vth = 0.5;
  else if(msg.angular.z < 0)
    vth = -0.5;
  if(msg.angular.z == 0)
    vth = 0;  
}
*/

void imu_callback(const sensor_msgs::Imu msg)
{
    quat = msg.orientation;
}

void wheel_encoder_callback(const std_msgs::String msg)
{
    left_encoder_prev = left_encoder;
    right_encoder_prev = right_encoder;

    int left;
    int right;

    string m = msg.data;

    stringstream ss(m);
    ss >> left;
    ss >> right;

    left_encoder = left;
    right_encoder = right;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;

    //initialise the publisher  
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    //ros::Subscriber sub = n.subscribe("cmd_vel", 10, cmd_vel_callback);
    ros::Subscriber enc_sub = n.subscribe("encoder_values", 10, wheel_encoder_callback);

    //subscriber for filtered imu data from imu_filter_madgwick node
    ros::Subscriber imu_sub = n.subscribe("imu/data", 10, imu_callback);

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;


    double vx = 0;
    double vy = 0;
    double vth = 0;

    double d_left_encoder;
    double d_right_encoder;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    //min and max values of encoder
    int max_val = 32768;
    int min_val = -32768;
    
    ros::Rate r(100);
    while(n.ok())
    {

        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();   

        //find delta of encoder readings and consider overflow/underflow
        if (left_encoder_prev > left_encoder)
        	d_left_encoder = (left_encoder - left_encoder_prev) % max_val;
        else if (left_encoder_prev < left_encoder)
        	d_left_encoder = (left_encoder_prev - left_encoder) % max_val;

        if (right_encoder_prev > right_encoder)
        	d_right_encoder = (right_encoder - right_encoder_prev) % max_val;
        else if (left_encoder_prev < left_encoder)
       		d_right_encoder = (right_encoder_prev - right_encoder) % max_val;


		//calculate velocities of each wheel
        double dt = (current_time - last_time).toSec();
        double l_vel = DISTANCE_PER_TICK * d_left_encoder / dt;
        double r_vel = DISTANCE_PER_TICK * d_right_encoder / dt;

        vx = (l_vel + r_vel) / 2;
        vy = 0;
        vth = -(r_vel - l_vel) / CHASIS_LENGTH;

        //compute odometry in a typical way given the velocities of the robot
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;    
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        //getting rotation from encoders
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		//getting rotation from IMU
		//geometry_msgs::Quaternion odom_quat = quat;

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "robot_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}

