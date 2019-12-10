#!/usr/bin/env python
# simple tool to rotate the imu orientation in the message

import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import sys
import math
import numpy as np
import tf

pub = rospy.Publisher('/camera/imu_fixed_with_orientation_in_base', Imu, queue_size=10)
print "rotate_imu tool running"

def callback(msg):
    rospy.loginfo_throttle(10, "Rotating imu orientation")
    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]

    rot1_mat   = tf.transformations.quaternion_matrix(quat)
    rot_unit = tf.transformations.euler_matrix(math.pi/2.0,-math.pi/2.0 , 0)
    rot = np.dot(rot1_mat, rot_unit)
    quat = tf.transformations.quaternion_from_matrix(rot)

    m = Imu()
    m.header = msg.header
    m.header.frame_id = "odom"
    m.orientation.w =quat[3]
    m.orientation.x =quat[0]
    m.orientation.y =quat[1]
    m.orientation.z =quat[2]
    pub.publish(m)

def listener():
    rospy.init_node('fake_imu', anonymous=True)
    rospy.Subscriber("/camera/imu_fixed_with_orientation", Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()