#!/usr/bin/env python3
import math
import rospy

from geometry_msgs.msg import Transform, Twist, Point, Quaternion
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

def angle_diff(a1, a2):
    diff = a1 - a2
    return (diff + math.pi) % (2 * math.pi) - math.pi

def summ(array):
    s = 0.
    for i in array:
        s += i
    return s

class Actions(object):
    def __init__(self):
        
        self.velocity_msg = Twist()
        self.velocity_pub = rospy.Publisher('mobile_base_controller/cmd_vel',
                                            Twist,
                                            queue_size=1)
        
        self.cam_pos_msg = Float64()
        self.cam_pos_pub = rospy.Publisher('camera_yaw_controller/command',
                                            Float64,
                                            queue_size=1)
        
        self.error_cam_len = 5
        self.error_body_len = 5
        
        self.error_cam_angle = [0.] * self.error_cam_len
        self.error_body_angle = [0.] * self.error_body_len

    def cam_pid(self, err):
        del self.error_cam_angle[0]
        self.error_cam_angle.append(err)
        p = 0.35
        i = 0.3
        d = 0.5
        dt = 0.1
        
        pp = p * self.error_cam_angle[-1]
        ii = i * self.error_cam_len * dt * summ(self.error_cam_angle)
        dd = d * (self.error_cam_angle[-1] - self.error_cam_angle[-2]) / dt
        
        #rospy.logerr("cam_pid: err: {:.6f}; p: {:.6f}; i: {:.6f}; d: {:.6f}".format(err, pp, ii, dd))
        signal = pp + ii + dd
        return signal
    
    def rot_pid(self, err):
        del self.error_body_angle[0]
        self.error_body_angle.append(err)
        p = 0.25
        i = 0.3
        d = 0.5
        dt = 0.1
        
        pp = p * self.error_body_angle[-1]
        ii = i * self.error_body_len * dt * summ(self.error_body_angle)
        dd = d * (self.error_body_angle[-1] - self.error_body_angle[-2]) / dt
        
        #rospy.logerr("rot_pid: err: {:.6f}; p: {:.6f}; i: {:.6f}; d: {:.6f}".format(err, pp, ii, dd))
        signal = pp + ii + dd
        return signal
    
    def act_accordingly(self, state, to_marker_angle, to_target_angle, cam_angle, target_ori, my_ori):
    #выбор действия в зависимости от состояния
        cam_err = angle_diff(to_marker_angle, cam_angle)
        #rospy.logerr("to marker angle: {:.3f}; cam angle: {:.3f}".format(to_marker_angle, cam_angle))
        if state == 'init':
            pass
        elif state == 'search':
            pass
            #крутить камерой и собой туда-сюда пока не наступит конец света
            
        elif state == 'aiming':
        #Вращать основание. Камера смотрит прямо на цель
            self.cam_pos_msg.data = cam_angle + self.cam_pid(cam_err)
            self.cam_pos_pub.publish(self.cam_pos_msg)
            
            self.velocity_msg.angular.z = self.rot_pid(to_target_angle)
            
            self.velocity_msg.linear.x = 0.
            
            self.velocity_pub.publish(self.velocity_msg)
            
        elif state == 'steering':
        #подруливать на ходу
            self.cam_pos_msg.data = cam_angle + self.cam_pid(cam_err)
            self.cam_pos_pub.publish(self.cam_pos_msg)
            
            self.velocity_msg.angular.z = self.rot_pid(to_target_angle)
            self.velocity_msg.linear.x = 0.25 #сделать регулируемой
            self.velocity_pub.publish(self.velocity_msg)
            
        elif state == 'moving':
        #движение вперёд (зачем?)
        
            self.velocity_msg.angular.z = 0
            self.velocity_msg.linear.x = 0.4
            self.velocity_pub.publish(self.velocity_msg)
            
        
        elif state == 'following':
        #преследовать
            self.cam_pos_msg.data = cam_angle + self.cam_pid(cam_err)
            self.cam_pos_pub.publish(self.cam_pos_msg)
            
