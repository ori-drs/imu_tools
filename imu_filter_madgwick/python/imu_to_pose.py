#!/usr/bin/env python
# publish imu orientation as a pose

import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
import sys
import math
import numpy as np
import tf

pub = rospy.Publisher('/state_estimator/pose_in_odom', PoseWithCovarianceStamped, queue_size=10)
print "rotate_imu tool running"
pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=10)

def callback(msg):
    print "got imu"
    # Offset to put the cloud in a different place than the map:
    x_offset =120
    y_offset = 0
    p = PoseWithCovarianceStamped()
    p.header = msg.header
    p.header.frame_id = "map"

    p.pose.pose.orientation = msg.orientation
    p.pose.pose.position.x = x_offset
    p.pose.pose.position.y = y_offset

    pub.publish(p)

    tfmsg = TFMessage()
    transformS = TransformStamped()

    transformS.header = msg.header
    transform = Transform()
    transform.rotation = msg.orientation
    transform.translation.x = x_offset
    transform.translation.y = y_offset
    transformS.transform = transform
    transformS.child_frame_id="base"
    tfmsg.transforms.append(transformS)    
    pub_tf.publish(tfmsg)

def listener():
    rospy.init_node('imu_to_pose', anonymous=True)
    rospy.Subscriber("/os1_cloud_node/imu_with_orientation", Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
