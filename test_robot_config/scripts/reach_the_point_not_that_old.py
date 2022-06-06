#!/usr/bin/env python3
import math

import rospy

from geometry_msgs.msg import Twist, Point
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

def angle_diff(a1, a2):
    diff = a1 - a2
    return (diff + math.pi) % (2 * math.pi) - math.pi

class FSM(object):
    def __init__(self):
        
        self.robot_link = rospy.get_param('~robot_link', 'test_robot::base_link')        
    #координаты и ориентация робота в odom
        self.body_abs_x = 0.
        self.body_abs_y = 0.
        self.body_abs_angle = 0.
    #координаты цели в системе координат робота
        self.dest_rel_x = None
        self.dest_rel_y = None
    #координаты цели в odom
        self.dest_abs_x = None
        self.dest_abs_y = None
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
        self.dest_sub = rospy.Subscriber('/destination', Point, self.cb_dest) # теперь тут координаты в odom
        
        self.cycles = 0
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_timer)
        
        self.error_cam_len = 5
        self.error_body_len = 5
        
        self.error_cam_angle = [0.] * self.error_cam_len
        self.error_body_angle = [0.] * self.error_body_len
        
    def cam_pid(self, err):
        del self.error_cam_angle[0]
        self.error_cam_angle.append(err)
        p = 0.25
        i = 0.5
        d = 0.5
        dt = 0.1
        
        pp = p * self.error_cam_angle[-1]
        ii = i * self.error_cam_len * dt * self.summ(self.error_cam_angle)
        dd = d * (self.error_cam_angle[-1] - self.error_cam_angle[-2]) / dt
        
        rospy.logerr("cam_pid: err: {:.6f}; p: {:.6f}; i: {:.6f}; d: {:.6f}".format(err, pp, ii, dd))
        signal = pp + ii + dd
        return signal
    
    def rot_pid(self, err):
        del self.error_body_angle[0]
        self.error_body_angle.append(err)
        p = 0.25
        i = 0.5
        d = 0.5
        dt = 0.1
        
        pp = p * self.error_body_angle[-1]
        ii = i * self.error_body_len * dt * self.summ(self.error_body_angle)
        dd = d * (self.error_body_angle[-1] - self.error_body_angle[-2]) / dt
        
        rospy.logerr("rot_pid: err: {:.6f}; p: {:.6f}; i: {:.6f}; d: {:.6f}".format(err, pp, ii, dd))
        signal = pp + ii + dd
        return signal
    
    def summ(self, array):
        s = 0.
        for i in array:
            s += i
        return s
        
    def goal_dir(self):
    #относительное направление на цель
        #rospy.logwarn("\tdest_rel_x:   {:.6f}; dest_rel_y:     {:.6f}"
                      #.format(self.dest_rel_x, self.dest_rel_y))
       # rospy.logwarn("\tcamera_angle: {:.6f}; body_abs_angle: {:.6f}"
                     # .format(self.camera_angle, self.body_abs_angle))
        #dir = angle_diff(math.atan2(self.dest_rel_y, self.dest_rel_x) + self.camera_angle, self.body_abs_angle)
        #rospy.logwarn("goal_dir: {:.6f}".format(dir))
        return math.atan2(self.dest_rel_y, self.dest_rel_x)
    
    def cb_odom(self, msg):
        self.body_abs_x = msg.pose.pose.position.x
        self.body_abs_y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.body_abs_angle = math.atan2(z, w) # а 2 зачем
    
    def cb_dest(self, msg):
        self.dest_abs_x = msg.x
        self.dest_abs_y = msg.y
        
        diff_x = self.dest_abs_x - self.body_abs_x
        diff_y = self.dest_abs_y - self.body_abs_y
        
        self.dest_rel_x =   diff_x * math.cos(self.body_abs_angle) + diff_y * math.sin(self.body_abs_angle)
        self.dest_rel_y = - diff_x * math.sin(self.body_abs_angle) + diff_y * math.cos(self.body_abs_angle)
        
        #rospy.logwarn("dest_rel_x: {:.6f}; dest_rel_y{:.6f}".format(self.dest_rel_x, self.dest_rel_y))
        return
    
    def cb_cam_pos(self, msg):
    #чтение позиции камеры
        try:
            i = msg.name.index('camera_joint')
        except ValueError:
            return
        self.camera_angle = msg.position[i]
        
    def act_accordingly(self):
    #действие в соответствии с состоянием
        if self.current_state == 'init':
            pass
        elif self.current_state == 'search':
            pass
            #крутить камерой и собой туда-сюда пока не наступит конец света
            
        elif self.current_state == 'aiming':
        #Вращать основание. Камера смотрит прямо на цель
            #self.cam_pos_msg.data = self.goal_dir()
            try:
                self.cam_pos_msg.data = self.camera_angle + self.cam_pid(angle_diff(math.atan2(self.dest_rel_y, self.dest_rel_x), self.camera_angle))
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
                self.cam_pos_msg.data = self.camera_angle + self.cam_pid(math.atan2(self.dest_rel_y, self.dest_rel_x))
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
    
    def do_i_see(self):
        if self.dest_abs_x is None:
            return False
        else:
            return True
    def do_i_stare(self):
    #True, если направление камеры на цель примерно ноль
        if abs(math.atan2(self.dest_rel_y, self.dest_rel_x)) < self.SMALL_ANGLE:
        #разность этих углов - направление цели в поле зрения камеры
            #rospy.logerr("camera directed at cycle: {}".format(self.cycles))
            return True
        else:
            return False
        
    def is_it_zero(self):
    #True, если позиция камеры примерно ноль
        if abs(self.camera_angle) < self.SMALL_ANGLE:
            #rospy.logerr("camera placed at cycle {}".format(self.cycles))
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
        if math.hypot(self.dest_rel_x, self.dest_rel_y) < 0.2:
            return True
        else:
            return False
    
    def cb_timer(self, e): 
    #обновление флагов + смена состояния
        self.is_goal_visible  = self.do_i_see()
        self.cycles += 1
        #rospy.logerr('fsm: do i see: {}'.format(self.is_goal_visible))
        try:
            self.is_camera_stares = self.do_i_stare()
            self.is_camera_placed = self.is_it_zero()
            self.is_body_directed = self.may_i_go()
            self.is_goal_reached  = self.is_it_near()
        except TypeError:
            pass
        #rospy.logerr('fsm: current state is {}'.format(self.current_state))
        self.previous_state = self.current_state
        self.switch_state()
        if self.previous_state != self.current_state:
            rospy.logerr('flags: {} {} {} {} {}; state: {}'
                        .format(self.is_goal_visible, self.is_camera_stares, self.is_camera_placed, self.is_body_directed, self.is_goal_reached, self.current_state))
            #self.cycles = 0
        self.act_accordingly()
        
        #try:
            #rospy.logerr('ori: {:.6f}; cam: {:.6f}; goal: {:.6f}'
                     #.format(self.body_abs_angle, self.camera_angle, self.goal_dir()))
        #except TypeError:
            #pass
    
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
                #self.current_state = 'steering'
                return
            if self.is_goal_reached:
                #self.current_state = 'stop'
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
                
    
if __name__ == '__main__':
    try:
        rospy.init_node('reach_the_point')
        f_s_m = FSM()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
    
