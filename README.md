# my_robot_software_stack
 EKF software stack for the DEWA R&D robot
 
 # **Setup**  
 
  ## **Prerequisites**  
  
  This project is using ROS Kinetic (Catkin also required) on Ubuntu 16.  
  
  ## **Installation**
  
  ### 1. Create a catkin workspace
  
  ``source /opt/ros/kinetic/setup.bash``
  
  ``mkdir -p ~/catkin_ws/src/   ``
  
  ``cd ~/catkin_ws/``  

 ### 2. Clone the repository into your catkin workspace's source folder  
 
 ``cd ~/catkin_ws/src/``
 
 ``git clone https://github.com/F-Sayed/my_robot_ekf``
 
 ### 3. Compile the workspace
 
 ``cd ~/catkin_ws``
 
 ``catkin_make``
 
 ## **Running the project**

Make sure all scripts that are being used have the execute permission by using ``chmod +x example_name.py``.  

``./catkin_ws/src/my_robot/scripts/launch_demo.sh``  

 
# **Node overview**

## **1. Keyboard Teleop**

The keyboard teleop node takes in keyboard commands to control the robot’s movements.

**Subscribes to:**

+ Keyboard commands (UIOJKLM,.) 

**Publishes to:**

+ /cmd_vel topic (geometry_msgs/Twist)

**c. Notes**

+ None


## **2. base_controller**

Translates movement messages to control the wheels of the robot.

**Subscribes to:**

+ /cmd_vel topic (geometry_msgs/Twist)

**Publishes to**

+ /encoder_values topic (std_msgs/String)

**c. Notes**

+ Need to convert linear velocity (x) and angular velocity (z) from cmd_vel to appropriate wheel speeds of the robot.

+ TODO: Need to change string message type to something more appropriate.


## **3. imu_raw_publisher**
  
+ Publishes raw IMU data as two messages, one for accelerometer and gyroscope values, another for magnetometer values.

**a. Subscribes to**

+ None

**b. Publishes to**

+ /imu/data_raw topic (sensor_msgs/Imu)

+ /imu/mag topic (sensor_msgs/MagneticField)

**c. Notes**

+ These values will be passed through a madgwick filter. 

+ Magnetometer values are used in this implementation but are optional.

+ IMU message takes in linear acceleration (accelerometer) and angular velocity (gyroscope) values.

+ Since the IMU publishes digital data, we need to convert each sensor to analog data (e.g. unit G’s or m/s).  



## **4. ImuFilter**

This node comes from the [imu_filter_madgwick](http://wiki.ros.org/imu_filter_madgwick) ROS package, which takes in raw IMU data (without orientation)
and publishes a filtered IMU message (with orientation).

**a.	Subscribes to**

+ /imu/data_raw topic (sensor_msgs/Imu)

+ /imu/mag topic (sensor_msgs/MagneticField)

**b.	Publishes to**

+ /imu/data topic (sensor_msgs/Imu)

**c.	Notes**

+ The subscribed topics have no information regarding the orientation of the robot, whereas the published 
topic’s message contains both the IMU readings as well as the orientation.

+ As mentioned, the /imu/mag topic is used in this implementation, but is optional.  



## **5. odometry_publisher**

+ Calculates robot translation and rotation based on given inputs such as wheel encoder readings or IMU data.

**a.	Subscribes to**

+ /imu/data topic (sensor_msgs/Imu)

+ /encoder_values topic (std_msgs/String)

**b.	Publishes to**

+ /odom topic (nav_msgs/Odometry)

**c.	Notes**

+ Translation are (currently) calculated using some preset measurements of the robot such as base length or
wheel ticks per cycle, as well as encoder readings.

+ Rotation can be calculated using either encoder readings (angular z axis), or the IMU readings.  

## **6. ublox_gps**

+ This node comes from the [ublox](http://wiki.ros.org/ublox) ROS package, and is used to publish GPS messages using the GPS used in this robot.

## ** a. Subscribes to**

+ None

## ** b. Publishes to**

+ /gps/fix topic (sensor_msgs/NavSatFix)

## ** c. Notes**

+ This node requires the [rtcm_msgs](https://index.ros.org/r/rtcm_msgs/) ROS package that is included in this repository.

## **7. navsat_transform_node**

This node comes from the [robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) ROS package, 
which works in a (sort of) loop with the EKF node by taking in the predicted estimate of the robot odometry, IMU readings, and GPS 
readings to publish an odom message of the robots position consistent with the robot’s world frame.

**a.	Subscribes to**

+ /imu/data topic (sensor_msgs/Imu)

+ /gps/fix topic (sensor_msgs/NavSatFix)

+ /odometry/filtered topic (nav_msgs/Odometry)

**b.	Publishes to**

+ /odometry/gps topic (nav_msgs/Odometry)

+ Optional: /gps/filtered topic (sensor_msgs/NavSatFix)

**c.	Notes**

+ This node works side-by-side with the EKF node (more below).  


## **8. ekf_localization_node**

This node comes from the [robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) ROS package. 
The ekf_localization_node is an implementation of an extended Kalman filter. It uses an omnidirectional motion model to project 
the state forward in time, and corrects that projected estimate using perceived sensor data.

**a. Subscribes to**

+ /set_pose

+ /cmd_vel

+ /odom 

+ /imu/data

+ /gps/NavSatFix


**b. Publishes to**

+ /odometry/filtered (nav_msgs/Odometry)

+ /accel/filtered (geometry_msgs/AccelWithCovarianceStamped) (if enabled)

**c. Notes**

+ Make sure to consider the ROS [REP-105](https://www.ros.org/reps/rep-0105.html) and [REP-103](https://www.ros.org/reps/rep-0103.html) when preparing your sensor data.

+ navsat_transform_node needs the the odometry from your state estimate because it converts the GPS data into the coordinate frame of that message.

+ You can have multiple sensors feeding data to the EKF node, such as multiple odometry or IMUs.
