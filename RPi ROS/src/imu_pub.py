#!/usr/bin/env python
import rospy
import time
import sys
import qwiic_icm20948
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3

#scale readings from +/- 2^15 to +/-4
def scale(val):
    return float(val)/ pow(2, 13)

if __name__ == '__main__':                
    IMU = qwiic_icm20948.QwiicIcm20948()
    if IMU.connected == False:
        print('IMU connection error...')
        exit()
    
    IMU.begin()
    rospy.init_node('imu_raw_publisher', anonymous=True)
    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)

    
    rate = rospy.Rate(10)

    linear_acceleration = Vector3()
    angular_velocity = Vector3()
    mag_field = Vector3()
    
    mag = MagneticField()
    mag.magnetic_field_covariance = [0, 0, 0,
                                     0, 0, 0,
                                     0, 0, 0]
    imu_raw = Imu()
    imu_raw.angular_velocity_covariance = [0, 0, 0,
                                           0, 0, 0,
                                           0, 0, 0]
    imu_raw.linear_acceleration_covariance = [0, 0, 0,
                                              0, 0, 0,
                                              0, 0, 0]

    while not rospy.is_shutdown():
        if IMU.dataReady():
            IMU.getAgmt()
            #assuming +/- 2g's
            linear_acceleration.x = scale(IMU.axRaw) 
            linear_acceleration.y = scale(IMU.ayRaw)
            linear_acceleration.z = scale(IMU.azRaw)
            
            angular_velocity.x = scale(IMU.gxRaw)
            angular_velocity.y = scale(IMU.gyRaw)
            angular_velocity.z = scale(IMU.gzRaw)
            
            mag_field.x = scale(IMU.mxRaw)
            mag_field.y = scale(IMU.myRaw)
            mag_field.z = scale(IMU.mzRaw)
            
            imu_raw.linear_acceleration = linear_acceleration
            imu_raw.angular_velocity = angular_velocity
            
            now = rospy.Time.now()
            
            imu_raw.header.stamp = now
            mag.header.stamp = now
            
            mag.magnetic_field = mag_field
            
            rospy.loginfo(imu_raw)
            
            imu_pub.publish(imu_raw)
            mag_pub.publish(mag)            
        rate.sleep()
            
        print()
