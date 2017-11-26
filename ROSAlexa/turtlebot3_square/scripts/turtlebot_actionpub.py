#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import math

# init publisher topic

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
vel_msg = Twist()


def callback(data):
    if data.data == 'forward':
        move(0.2, 0.5, True)

        # rospy.loginfo("I yes heard %s",data.data)

    if data.data == 'backward':
        move(0.1, 0.5, False)

        # rospy.loginfo("yo yo backward")

    if data.data == 'square':
        square(0.3)

        # rospy.loginfo('heard %s',data.data)

    if data.data == 'stop':
        move(0, 0, False)


        # rospy.loginfo('heard %s',data.data)

def move(speed, distance, isForward):
    if isForward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    t0 = rospy.get_rostime()
    current_dist = 0.0

    rate = rospy.Rate(10)  # 10 messages per s
    while current_dist < distance:
        pub.publish(vel_msg)
        t1 = rospy.get_rostime()
        current_dist = speed * (t1.secs - t0.secs)

    # after we achieved the  stop the robot

    vel_msg.linear.x = 0
    pub.publish(vel_msg)


def rotate(angular_speed, angle, isClockwise):
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    if isClockwise:
        vel_msg.angular.z = abs(angular_speed)
    else:
        vel_msg.angular.z = -abs(angular_speed)

    t0 = rospy.get_rostime()
    current_angle = 0.0

    rate = rospy.Rate(10)  # 10 messages per sec

    while current_angle < angle:
        pub.publish(vel_msg)
        t1 = rospy.get_rostime()
        current_angle = angular_speed * (t1.secs - t0.secs)

    vel_msg.angular.z = 0
    pub.publish(vel_msg)


def square(speed):
    move(0.2, 0.5, True)
    time.sleep(2)
    rotate(math.radians(10), math.radians(90), False)
    time.sleep(2)

    move(0.2, 0.5, True)
    time.sleep(2)
    rotate(math.radians(10), math.radians(90), False)
    time.sleep(2)

    move(0.2, 0.5, True)
    time.sleep(2)
    rotate(math.radians(10), math.radians(90), False)
    time.sleep(2)

    move(0.2, 0.5, True)


if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)

    rospy.spin()
