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

class FSM(object):
    def __init__(self):
        
        self.robot_link = rospy.get_param('~robot_link', 'test_robot::base_link')        
    #координаты и ориентация робота в odom
        self.body_odom_x = 0.
        self.body_odom_y = 0.
        self.body_odom_angle = 0.
    #разность абсолютных координат цели и робота
        self.marker_minus_robot_x = None
        self.marker_minus_robot_y = None
    #координаты цели в odom
        self.marker_odom_x = None
        self.marker_odom_y = None
        self.marker_odom_angle = None
    # угол поворота камеры в системе координат робота
        self.camera_angle = 0.   
    #задаётся публикацией в camera_yaw_controller/command
    #узнаётся из /joint_states - зачем, если я же его и задала?
        self.SMALL_ANGLE = 0.05
        self.BIGGER_ANGLE = 0.4
        
        
        self.is_goal_visible  = False
        self.is_camera_stares = False   # камера смотрит прямо на цель
        self.is_camera_placed = False   # камера повёрнута на 0 градусов
    # два флага выше истинны <=> тело направлено точно на цель
        self.is_body_directed = False   # тело направлено почти на цель
        self.is_goal_reached  = False
        
        self.current_state = 'init'
            #init
            #search
            #aiming
            #steering
            #moving
            #stop
        
        self.velocity_msg = Twist()
        self.velocity_pub = rospy.Publisher('mobile_base_controller/cmd_vel',
                                            Twist,
                                            queue_size=1)
        
        self.cam_pos_msg = Float64()
        self.cam_pos_pub = rospy.Publisher('camera_yaw_controller/command',
                                            Float64,
                                            queue_size=1)
                                            
        self.odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.cb_odom)
        self.cam_pos_sub = rospy.Subscriber('/joint_states', JointState, self.cb_cam_pos)
        self.dest_sub = rospy.Subscriber('/destination', Transform, self.cb_dest) # теперь тут координаты в odom
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_timer)
        
        self.error_cam_len = 5
        self.error_body_len = 5
        
        self.error_cam_angle = [0.] * self.error_cam_len
        self.error_body_angle = [0.] * self.error_body_len
        
    def cb_odom(self, msg):
        
        self.body_odom_x = msg.pose.pose.position.x
        self.body_odom_y = msg.pose.pose.position.y
        
        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]
        self.body_odom_angle = euler_from_quaternion(quat)[2] #см индекс
        #почему
        
        return
        
    def cb_cam_pos(self, msg):
    #чтение позиции камеры
        try:
            i = msg.name.index('camera_joint')
        except ValueError:
            return
        self.camera_angle = msg.position[i]
        
        return
    
    def cb_dest(self, msg):
    #определить координаты цели в системе координат робота
        self.marker_odom_x = msg.translation.x
        self.marker_odom_y = msg.translation.y
        
        quat = [msg.rotation.x,
                msg.rotation.y,
                msg.rotation.z,
                msg.rotation.w]
        self.marker_odom_angle = euler_from_quaternion(quat)[2]
        
        self.marker_minus_robot_x = self.marker_odom_x - self.body_odom_x
        self.marker_minus_robot_y = self.marker_odom_y - self.body_odom_y
        #rospy.logerr("\tx\ty")
        #rospy.logerr("dest\t{:.3f}\t{:.3f}".format(self.marker_odom_x, self.marker_odom_y))
        #rospy.logerr("body\t{:.3f}\t{:.3f}".format(self.body_odom_x, self.body_odom_y))
        #rospy.logerr("diff\t{:.3f}\t{:.3f}".format(self.marker_minus_robot_x, self.marker_minus_robot_y))
        return
    
    def goal_dir(self):
        #rospy.logerr("goal_dir: marker_minus_robot_x is {:.6f}; marker_minus_robot_y is {:.6f}"
                     #.format(self.marker_minus_robot_x, self.marker_minus_robot_y))
        #rospy.logerr("goal_dir: abs direction is {:.6f}; body direction is {:.6f}"
                     #.format(math.atan2(self.marker_minus_robot_y, self.marker_minus_robot_x), self.body_odom_angle))
     
        return angle_diff(math.atan2(self.marker_minus_robot_y, self.marker_minus_robot_x), self.body_odom_angle) 
    
    def do_i_see(self):
        if self.marker_odom_x is None:
            return False
        else:
            return True
    def do_i_stare(self):
    #True, если направление камеры на цель примерно ноль
        if abs(angle_diff(self.goal_dir(), self.camera_angle)) < self.SMALL_ANGLE:
            return True
        else:
            return False
        
    def is_it_zero(self):
    #True, если позиция камеры примерно ноль
        if abs(self.camera_angle) < self.SMALL_ANGLE:
            return True
        else:
            return False
    
    def may_i_go(self):
    #True, если тело +- наведено
        if abs(self.goal_dir()) < self.BIGGER_ANGLE:
            return True
        else:
            return False
    
    def is_it_near(self):
    #цель условно достигнута
        if math.hypot(self.marker_minus_robot_x, self.marker_minus_robot_y) < 0.2:
            return True
        else:
            return False
    
    def switch_state(self):
        if self.current_state == 'init':
            if self.is_goal_reached:
                self.current_state = 'stop'
                return
                
            if self.is_goal_visible:
                self.current_state = 'aiming'
                return
            else:
                self.current_state = 'search'
                return
        elif self.current_state == 'search':
            if self.is_goal_visible:
                self.current_state = 'aiming'
                return
        elif self.current_state == 'aiming':
            if not self.is_goal_visible:
                self.current_state = 'search'
                return
                
            if self.is_body_directed:
                self.current_state = 'steering'
                return
            if self.is_goal_reached:
                self.current_state = 'stop'
                return
            return
        elif self.current_state == 'steering':
            if not self.is_goal_visible:
                self.current_state = 'search'
                return
            if not self.is_body_directed:
                self.current_state = 'aiming'
                return
            if self.is_camera_stares and self.is_camera_placed:
                self.current_state = 'moving'
                return
            if self.is_goal_reached:
                self.current_state = 'stop'
                return
            return
        elif self.current_state == 'moving':
            if not self.is_goal_visible:
                self.current_state = 'search'
                return
                
            if  (not self.is_camera_stares) or (not self.is_camera_placed):
                self.current_state = 'steering'
                return
            if self.is_goal_reached:
                self.current_state = 'stop'
            return 
        #'stop'
    
    def summ(self, array):
        s = 0.
        for i in array:
            s += i
        return s
    
    def cam_pid(self, err):
        del self.error_cam_angle[0]
        self.error_cam_angle.append(err)
        p = 0.25
        i = 0.3
        d = 0.5
        dt = 0.1
        
        pp = p * self.error_cam_angle[-1]
        ii = i * self.error_cam_len * dt * self.summ(self.error_cam_angle)
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
        ii = i * self.error_body_len * dt * self.summ(self.error_body_angle)
        dd = d * (self.error_body_angle[-1] - self.error_body_angle[-2]) / dt
        
        #rospy.logerr("rot_pid: err: {:.6f}; p: {:.6f}; i: {:.6f}; d: {:.6f}".format(err, pp, ii, dd))
        signal = pp + ii + dd
        return signal
    
    def act_accordingly(self):
    #выбор действия в зависимости от состояния
        if self.current_state == 'init':
            pass
        elif self.current_state == 'search':
            pass
            #крутить камерой и собой туда-сюда пока не наступит конец света
            
        elif self.current_state == 'aiming':
        #Вращать основание. Камера смотрит прямо на цель
            try:
                
                self.cam_pos_msg.data = self.camera_angle + self.cam_pid(angle_diff(self.goal_dir(), self.camera_angle))
                self.cam_pos_pub.publish(self.cam_pos_msg)
            except TypeError:
                pass
            self.velocity_msg.angular.z = self.rot_pid(self.goal_dir())
            
            self.velocity_msg.linear.x = 0.
            self.velocity_pub.publish(self.velocity_msg)
            
        elif self.current_state == 'steering':
        #подруливать на ходу
            #self.cam_pos_msg.data = self.goal_dir()
            try:
                self.cam_pos_msg.data = self.camera_angle + self.cam_pid(angle_diff(self.goal_dir(), self.camera_angle))
                self.cam_pos_pub.publish(self.cam_pos_msg)
            except TypeError:
                pass
            self.velocity_msg.angular.z = self.rot_pid(self.goal_dir())
            self.velocity_msg.linear.x = 0.05
            self.velocity_pub.publish(self.velocity_msg)
            
        elif self.current_state == 'moving':
        #полный вперёд
        
            self.velocity_msg.angular.z = 0
            self.velocity_msg.linear.x = 0.1
            self.velocity_pub.publish(self.velocity_msg)
    
    def cb_timer(self, e):
    #обновление флагов + смена состояния
        self.is_goal_visible  = self.do_i_see()
        
        try:
            self.is_camera_stares = self.do_i_stare()
            self.is_camera_placed = self.is_it_zero()
            self.is_body_directed = self.may_i_go()
            self.is_goal_reached  = self.is_it_near()
            
            rospy.logwarn("body: {:.3f}\tmarker: {:.3f}".format(self.body_odom_angle, self.marker_odom_angle))
        except TypeError:
            pass
        
        self.previous_state = self.current_state
        self.switch_state()
        if self.previous_state != self.current_state:
            rospy.logerr('flags: {} {} {} {} {}; state: {}'
                        .format(self.is_goal_visible, self.is_camera_stares, self.is_camera_placed, self.is_body_directed, self.is_goal_reached, self.current_state))
        self.act_accordingly()
        return

if __name__ == '__main__':
    try:
        rospy.init_node('reach_the_point')
        f_s_m = FSM()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
