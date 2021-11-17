# my_robot_software_stack
 EKF software stack for the DEWA R&D robot
 
Node overview

**1.	Keyboard Teleop**

Takes in keyboard commands to control the robot’s movements.
 
**a.	Subscribes to**

  Keyboard commands (UIOJKLM,.) 

**b.	Publishes to**

  /cmd_vel topic (geometry_msgs/Twist)
  
**c.	Notes**

**2.	base_controller**

Translates movement messages to control the wheels of the robot.

**a.	Subscribes to**

  /cmd_vel topic (geometry_msgs/Twist)
  
  **b.	Publishes to**
  
  /encoder_values topic (std_msgs/String)

  **c.	Notes**

  Need to convert linear velocity (x) and angular velocity (z) from cmd_vel to appropriate wheel speeds of the robot.

  TODO: Need to change string message type to something more appropriate.

  **3.	imu_raw_publisher**
  
Publishes raw IMU data as two messages, one for accelerometer and gyroscope values, another for magnetometer values.

**a.	Subscribes to**

None

**b.	Publishes to**

/imu/data_raw topic (sensor_msgs/Imu)

/imu/mag topic (sensor_msgs/MagneticField)

**c.	Notes**

These values will be passed through a madgwick filter. 

Magnetometer values are used in this implementation but are optional.

IMU message takes in linear acceleration (accelerometer) and angular velocity (gyroscope) values.

Since the IMU publishes digital data, we need to convert each sensor to analog data (e.g. unit G’s or m/s).

**4.	ImuFilter**

This node comes from the imu_filter_madgwick ROS package, which takes in raw IMU data (without orientation) and publishes a filtered IMU message (with orientation).

**a.	Subscribes to**

/imu/data_raw topic (sensor_msgs/Imu)

/imu/mag topic (sensor_msgs/MagneticField)

**b.	Publishes to**

/imu/data topic (sensor_msgs/Imu)

**c.	Notes**

The subscribed topics have no information regarding the orientation of the robot, whereas the published topic’s message contains both the IMU readings as well as the orientation.

As mentioned, the /imu/mag topic is used in this implementation, but is optional.

**5.	odometry_publisher**

Calculates robot translation and rotation based on given inputs such as wheel encoder readings or IMU data.

**a.	Subscribes to**

/imu/data topic (sensor_msgs/Imu)

/encoder_values topic (std_msgs/String)

**b.	Publishes to**

/odom topic (nav_msgs/Odometry)

**c.	Notes**

Translation are (currently) calculated using some preset measurements of the robot such as base length or wheel ticks per cycle, as well as encoder readings.

Rotation can be calculated using either encoder readings (angular z axis), or the IMU readings.

**6.	navsat_transform_node**

This node comes from the robot_localization ROS package, which works in a (sort of) loop with the EKF node by taking in the predicted estimate of the robot odometry, IMU readings, and GPS readings to publish an odom message of the robots position consistent with the robot’s world frame.

**a.	Subscribes to**

/imu/data topic (sensor_msgs/Imu)

/gps/fix topic (sensor_msgs/NavSatFix)

/odometry/filtered topic (nav_msgs/Odometry)

**b.	Publishes to**

/odometry/gps topic (nav_msgs/Odometry)

Optional: /gps/filtered topic (sensor_msgs/NavSatFix)

**c.	Notes**

This node works side-by-side with the EKF node (more below).

