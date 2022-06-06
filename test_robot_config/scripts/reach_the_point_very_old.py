#!/usr/bin/env python3
import math

import rospy

from geometry_msgs.msg import Twist,Point
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

def angle_diff(a1, a2):
    diff = a1 - a2
    return (diff + math.pi) % (2 * math.pi) - math.pi

class FSM(object):
    def __init__(self):
        
        self.robot_link = rospy.get_param('~robot_link', 'test_robot::base_link')
        
        self.x = 0.
        self.y = 0.
        self.angle = 0.          # угол поворота тела в глобальной системе координат
        self.camera_angle = 0.   # угол поворота камеры в глобальной системе координат
        self.SMALL_ANGLE = 0.05
        self.BIGGER_ANGLE = 0.4
        
        self.goal_x = 3.
        self.goal_y = -1.
        
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
                                            
        self.joints_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.cb_link_states)
        self.cam_pos_sub = rospy.Subscriber('/joint_states', JointState, self.cb_cam_pos)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_timer)
        #rospy.logerr('!'*100)
        
    def goal_dir(self):
    #относительное направление на цель
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        almost_angle = math.atan2(dy, dx)
        return angle_diff(almost_angle, self.angle)
    
    def cb_cam_pos(self, msg):
    #чтение позиции камеры
        try:
            i = msg.name.index('camera_joint')
        except ValueError:
            return
        self.camera_angle = msg.position[i]
        #rospy.logerr('camera_joint: camera_angle: {:.3f}'.format(self.camera_angle))
        
    def cb_link_states(self, msg):
    #чтение координат и ориентации робота
        try:
            i = msg.name.index('test_robot::base_link')
        except ValueError:
            return
        self.x = msg.pose[i].position.x
        self.y = msg.pose[i].position.y
        #rospy.logerr('test_robot::base_link: position: {:.3f} {:.3f}'.format(self.x, self.y))
        z = msg.pose[i].orientation.z
        w = msg.pose[i].orientation.w
        self.angle = 2 * math.atan2(z, w)
        
    def act_accordingly(self):
    #действие в соответствии с состоянием
        if self.current_state == 'init':
            pass
        elif self.current_state == 'search':
            pass
            #крутить камерой и собой туда-сюда пока не наступит конец света
            
        elif self.current_state == 'aiming':
        #Вращать основание. Камера смотрит прямо на цель
            self.cam_pos_msg.data = self.goal_dir()
            self.cam_pos_pub.publish(self.cam_pos_msg)
            self.velocity_msg.angular.z = 0.5 if self.goal_dir() > 0 else -0.5
            self.velocity_msg.linear.x = 0
            self.velocity_pub.publish(self.velocity_msg)
            
        elif self.current_state == 'steering':
        #подруливать на ходу
            self.cam_pos_msg.data = self.goal_dir()
            self.cam_pos_pub.publish(self.cam_pos_msg)
            self.velocity_msg.angular.z = 0.2 if self.goal_dir() > 0 else -0.05
            self.velocity_msg.linear.x = 0.3
            self.velocity_pub.publish(self.velocity_msg)
            
        elif self.current_state == 'moving':
        #полный вперёд
            self.velocity_msg.angular.z = 0
            self.velocity_msg.linear.x = 0.5
            self.velocity_pub.publish(self.velocity_msg)
    
    def do_i_see(self):
        return True
        #вычислить расстояние и направление до цели, если это возможно
    def do_i_stare(self):
    #True, если направление камеры на цель примерно ноль
        if abs(angle_diff(self.camera_angle, self.goal_dir())) < self.SMALL_ANGLE:
            return True
        else:
            return False
        
    def is_it_zero(self):
    #True, если позиция камеры примерно ноль
        if abs(self.camera_angle) < 0.01:
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
        if math.hypot(self.x - self.goal_x, self.y - self.goal_y) < 0.1:
            return True
        else:
            return False
    
    def cb_timer(self, e): 
    #обновление флагов + смена состояния
        self.is_goal_visible  = self.do_i_see()
        #rospy.logerr('fsm: do i see: {}'.format(self.is_goal_visible))
        self.is_camera_stares = self.do_i_stare()
        self.is_camera_placed = self.is_it_zero()
        self.is_body_directed = self.may_i_go()
        self.is_goal_reached  = self.is_it_near()
        #rospy.logerr('fsm: current state is {}'.format(self.current_state))
        self.previous_state = self.current_state
        self.switch_state()
        if self.previous_state != self.current_state:
            rospy.logerr('flags: {} {} {} {} {}; state: {}'
                        .format(self.is_goal_visible, self.is_camera_stares, self.is_camera_placed, self.is_body_directed, self.is_goal_reached, self.current_state))
        self.act_accordingly()
        
        #rospy.logerr('ori: {:.6f}; cam: {:.6f}; goal: {:.6f}'
                     #.format(self.angle, self.camera_angle, self.goal_dir()))
    
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
                
    
if __name__ == '__main__':
    try:
        rospy.init_node('reach_the_point')
        f_s_m = FSM()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
    
