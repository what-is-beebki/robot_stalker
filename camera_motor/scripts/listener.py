#!/usr/bin/env python3

import math
import time
import camera_motor.maestro as maestro
import rospy
import serial

from std_msgs.msg import Float32

servo = None
servo_pin = 0
min_value = 2400
max_value = 9200

def callback(data):
    global servo
    
    x = int(data.data * (max_value - min_value) / math.pi + (max_value + min_value)/2)
    
    servo.setTarget(servo_pin, x)
    print('command: {}'.format(x))

def listener():
    global servo
    rospy.init_node('camera_motor', anonymous=True)


    # servo controller initialization
    try:
        servo = maestro.Controller(ttyStr='/dev/ttyACM0')
    except serial.serialutil.SerialException:
        rospy.logerr('camera_motor: device not found.')
        exit(-1)
    servo.setRange(servo_pin, min_value, max_value)

    rospy.Subscriber('camera_position', Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
