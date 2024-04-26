#!/usr/bin/env python3

import os
import time
import subprocess
import math

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

import RPi.GPIO as GPIO

class detect:
    def __init__(self):
        self.radar_open()

        self.distance = 0.4

        self.pub = rospy.Publisher('/obstacle', Bool, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.get_lidar_data)
        
    def get_lidar_data(self, msg):
        temp = dict()
        for i, v in enumerate(msg.ranges):
            direction = int(math.degrees(i*msg.angle_increment + msg.angle_min))
            temp[direction] = v if direction not in temp.keys() else max(v, temp[direction])
        temp = [v for k, v in temp.items()]
        temp = temp[:20] + temp[-20:]
        temp = [v for v in temp if 0 < v < self.distance]
        print(len(temp), ':', temp)
        #half of length of temp
        if len(temp) > 19:
            self.pub.publish(True)
        else:
            self.pub.publish(False)

    def radar_open(self):
        def radar_high():
            GPIO.setmode(GPIO.BCM)
            time.sleep(0.1)
            GPIO.setup(20, GPIO.OUT)
            GPIO.output(20, GPIO.HIGH)

        radar_high()
        time.sleep(0.05)

if __name__=='__main__':
    rospy.init_node('detect_front_obstacle')

    detect()

    rospy.spin()

    GPIO.cleanup()
