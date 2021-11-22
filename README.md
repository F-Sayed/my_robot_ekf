# my_robot_software_stack
 EKF software stack for the DEWA R&D robot
 
# **Node overview**

## **1. Keyboard Teleop**

The keyboard teleop node takes in keyboard commands to control the robot’s movements.

**Subscribes to:**

Keyboard commands (UIOJKLM,.) 

**Publishes to:**

/cmd_vel topic (geometry_msgs/Twist)

**--Notes--**

None


## **2. base_controller**

Translates movement messages to control the wheels of the robot.

**Subscribes to:**

- /cmd_vel topic (geometry_msgs/Twist)

**Publishes to**

- /encoder_values topic (std_msgs/String)

**--Notes--**

Need to convert linear velocity (x) and angular velocity (z) from cmd_vel to appropriate wheel speeds of the robot.

TODO: Need to change string message type to something more appropriate.


## **3. imu_raw_publisher**
  
Publishes raw IMU data as two messages, one for accelerometer and gyroscope values, another for magnetometer values.

**a. Subscribes to**

None

**b. Publishes to**

/imu/data_raw topic (sensor_msgs/Imu)

/imu/mag topic (sensor_msgs/MagneticField)

**c. Notes**

These values will be passed through a madgwick filter. 

Magnetometer values are used in this implementation but are optional.

IMU message takes in linear acceleration (accelerometer) and angular velocity (gyroscope) values.

Since the IMU publishes digital data, we need to convert each sensor to analog data (e.g. unit G’s or m/s).  



## **4. ImuFilter**

This node comes from the [imu_filter_madgwick](http://wiki.ros.org/imu_filter_madgwick) ROS package, which takes in raw IMU data (without orientation)
and publishes a filtered IMU message (with orientation).

**a.	Subscribes to**

/imu/data_raw topic (sensor_msgs/Imu)

/imu/mag topic (sensor_msgs/MagneticField)

**b.	Publishes to**

/imu/data topic (sensor_msgs/Imu)

**c.	Notes**

The subscribed topics have no information regarding the orientation of the robot, whereas the published 
topic’s message contains both the IMU readings as well as the orientation.

As mentioned, the /imu/mag topic is used in this implementation, but is optional.  



## **5. odometry_publisher**

Calculates robot translation and rotation based on given inputs such as wheel encoder readings or IMU data.

**a.	Subscribes to**

/imu/data topic (sensor_msgs/Imu)

/encoder_values topic (std_msgs/String)

**b.	Publishes to**

/odom topic (nav_msgs/Odometry)

**c.	Notes**

Translation are (currently) calculated using some preset measurements of the robot such as base length or
wheel ticks per cycle, as well as encoder readings.

Rotation can be calculated using either encoder readings (angular z axis), or the IMU readings.  



## **6. navsat_transform_node**

This node comes from the [robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) ROS package, which works in a (sort of) loop with the EKF node by taking in the
predicted estimate of the robot odometry, IMU readings, and GPS readings to publish an odom message of the robots position 
consistent with the robot’s world frame.

**a.	Subscribes to**

/imu/data topic (sensor_msgs/Imu)

/gps/fix topic (sensor_msgs/NavSatFix)

/odometry/filtered topic (nav_msgs/Odometry)

**b.	Publishes to**

/odometry/gps topic (nav_msgs/Odometry)

Optional: /gps/filtered topic (sensor_msgs/NavSatFix)

**c.	Notes**

This node works side-by-side with the EKF node (more below).  


## **7. ekf_localization_node**

This node comes from the [robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) ROS package. 
The ekf_localization_node is an implementation of an extended Kalman filter. It uses an omnidirectional motion model to project 
the state forward in time, and corrects that projected estimate using perceived sensor data.

**a. Subscribes to**

/set_pose

/cmd_vel

/odom 

/imu/data

/gps/NavSatFix


**b. Publishes to**

/odometry/filtered (nav_msgs/Odometry)

/accel/filtered (geometry_msgs/AccelWithCovarianceStamped) (if enabled)

**c. Notes**

Make sure to consider the ROS [REP-105](https://www.ros.org/reps/rep-0105.html) and [REP-103](https://www.ros.org/reps/rep-0103.html) when preparing your sensor data.

navsat_transform_node needs the the odometry from your state estimate because it converts the GPS data into the coordinate frame of that message.

