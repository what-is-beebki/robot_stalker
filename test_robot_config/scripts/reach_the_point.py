#!/usr/bin/env python3
import math
from numpy import array, dot
from fsm_description import FSM
from wheels_and_eye import Actions
import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped, Transform, Twist, Point, Quaternion

from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

def angle_diff(a1, a2):
    diff = a1 - a2
    return (diff + math.pi) % (2 * math.pi) - math.pi


class robot_stalker(object):
    def __init__(self):
        
        self.fsm = FSM()
        self.act = Actions()
        
        self.robot_link = rospy.get_param('~robot_link', 'test_robot::base_link')        
    #координаты и ориентация робота в odom
        self.body_odom_x = 0.
        self.body_odom_y = 0.
        self.body_odom_angle = 0.
    #разность координат цели и робота в odom
        self.marker_minus_robot_x = None
        self.marker_minus_robot_y = None
    #координаты маркера в odom
        self.marker_odom_x = None
        self.marker_odom_y = None
        self.marker_odom_angle = None
    #целевое положение в odom
        self.target_odom_x = None
        self.target_odom_y = None
    #разность координат целевого положения и робота
        self.target_minus_robot_x = None
        self.target_minus_robot_y = None
    # угол поворота камеры в системе координат робота
        self.camera_angle = 0.   
    #задаётся публикацией в camera_yaw_controller/command
    #вектор, на который робот смещается относительно цели при преследовании
        self.shift_vector = [rospy.get_param('~shift_back', 1.), rospy.get_param('~shift_right', 0.5)]
    #узнаётся из /joint_states - зачем, если я же его и задала?
        self.SMALL_ANGLE = 0.05 #погрешность наведения камеры
        self.BIGGER_ANGLE = 0.4 
        self.BIGGER_DIST = 0.2
        self.SMALL_DIST = 0.1   #погрешность положения тела - чтобы работала нормально, нужно настроить фильтр, чтобы он не скакал как чёрт
                                            
        self.odom_sub = rospy.Subscriber('mobile_base_controller/odom', Odometry, self.cb_odom)
        self.cam_pos_sub = rospy.Subscriber('joint_states', JointState, self.cb_cam_pos)
        self.dest_sub = rospy.Subscriber('destination', Transform, self.cb_dest) #положение и ориентация маркера в odom
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # посылаем в tf точку, в которой тобот должен находиться, чтобы преследовать маркер
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_timer)
        
    def cb_odom(self, msg):
        
        self.body_odom_x = msg.pose.pose.position.x
        self.body_odom_y = msg.pose.pose.position.y
        
        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]
        self.body_odom_angle = euler_from_quaternion(quat)[2] #yaw
        
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
    #определить координаты маркера и целевого положения в системе координат робота
        self.marker_odom_x = msg.translation.x
        self.marker_odom_y = msg.translation.y
        
        quat = [msg.rotation.x,
                msg.rotation.y,
                msg.rotation.z,
                msg.rotation.w]
        self.marker_odom_angle = euler_from_quaternion(quat)[2]
        theta = self.marker_odom_angle
        
        self.marker_minus_robot_x = self.marker_odom_x - self.body_odom_x
        self.marker_minus_robot_y = self.marker_odom_y - self.body_odom_y
        rot_matrix = array([[math.sin(theta), math.cos(theta)], 
                            [-math.cos(theta), math.sin(theta)]])
        rot_shift_vector = dot(rot_matrix, self.shift_vector)
        self.target_odom_x = self.marker_odom_x + rot_shift_vector[0]
        self.target_odom_y = self.marker_odom_y + rot_shift_vector[1]
        self.target_minus_robot_x = self.target_odom_x - self.body_odom_x
        self.target_minus_robot_y = self.target_odom_y - self.body_odom_y
        #rospy.logerr("\t\tx\ty")
        #rospy.logerr("marker\t{:.3f}\t{:.3f}".format(self.marker_odom_x, self.marker_odom_y))
        #rospy.logerr("shift\t{:.3f}\t{:.3f}".format(rot_shift_vector[0], rot_shift_vector[1]))
        #rospy.logerr("target\t{:.3f}\t{:.3f}".format(self.target_odom_x, self.target_odom_y))
        #rospy.logerr("body\t{:.3f}\t{:.3f}".format(self.body_odom_x, self.body_odom_y))
        #rospy.logerr("diff\t{:.3f}\t{:.3f}".format(self.target_minus_robot_x, self.target_minus_robot_y))
        t = TransformStamped()
                
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'odom'
        ns = rospy.get_namespace()
        if ns != '':
            t.child_frame_id = ns + 'target_pos'
        else:
            t.child_frame_id = 'target_pos'
        
        t.transform.translation.x = self.target_odom_x
        t.transform.translation.y = self.target_odom_y
        t.transform.rotation.w    = 1
        
        self.tf_broadcaster.sendTransform(t)
        
        return
    
    def marker_dir(self):
        #относительное направление робота на маркер
        #rospy.logerr("marker_dir: marker_minus_robot_x is {:.6f}; marker_minus_robot_y is {:.6f}"
                     #.format(self.marker_minus_robot_x, self.marker_minus_robot_y))
        #rospy.logerr("marker_dir: abs direction is {:.6f}; body direction is {:.6f}"
                     #.format(math.atan2(self.marker_minus_robot_y, self.marker_minus_robot_x), self.body_odom_angle))
     
        return angle_diff(math.atan2(self.marker_minus_robot_y, self.marker_minus_robot_x), self.body_odom_angle) 
    
    def target_dir(self):
        #относительное направление робота на целевое положение
     
        return angle_diff(math.atan2(self.target_minus_robot_y, self.target_minus_robot_x), self.body_odom_angle)
    
    def do_i_see(self):
        if self.marker_odom_x is None:
            return False
        else:
            return True
        
    def do_i_stare(self):
    #True, если направление камеры на цель примерно ноль
        if abs(angle_diff(self.marker_dir(), self.camera_angle)) < self.SMALL_ANGLE:
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
        if abs(self.target_dir()) < self.BIGGER_ANGLE:
            return True
        else:
            return False
    
    def is_it_near(self):
    #цель условно достигнута
        if math.hypot(self.target_minus_robot_x, self.target_minus_robot_y) < self.SMALL_DIST:
            return True
        else:
            return False
    def am_i_lost(self):
    #робот ориентирован правильно    
        if math.hypot(self.target_minus_robot_x, self.target_minus_robot_y) > self.BIGGER_DIST:
            return True
        else:
            return False
    def am_i_straight(self):
    #робот ориентирован правильно    
        if abs(angle_diff(self.body_odom_angle, self.marker_odom_angle + math.pi / 2)) < self.SMALL_ANGLE:
            return True
        else:
            return False
    def cb_timer(self, e):
    #обновление флагов + смена состояния
        self.fsm.is_goal_visible  = self.do_i_see()
        try:
            self.fsm.is_camera_stares = self.do_i_stare()
            self.fsm.is_camera_placed = self.is_it_zero()
            self.fsm.is_body_directed = self.may_i_go()
            self.fsm.is_goal_reached  = self.is_it_near()
            self.fsm.is_goal_far      = self.am_i_lost()
            self.fsm.is_ori_proper    = self.am_i_straight()
            
            #rospy.logwarn("body: {:.3f}\tmarker: {:.3f}".format(self.body_odom_angle, self.marker_odom_angle))
        except TypeError:
            pass
        
        self.fsm.previous_state = self.fsm.current_state
        self.fsm.switch_state()
        if self.fsm.previous_state != self.fsm.current_state:
            rospy.logerr('flags: {} {} {} {} {}; state: {}'
                        .format(self.fsm.is_goal_visible, self.fsm.is_camera_stares, self.fsm.is_camera_placed, self.fsm.is_body_directed, self.fsm.is_goal_reached, self.fsm.current_state))
        try:
            self.act.act_accordingly(self.fsm.current_state, self.marker_dir(), self.target_dir(), self.camera_angle, self.marker_odom_angle, self.body_odom_angle)
        except TypeError:
            self.act.act_accordingly(self.fsm.current_state, None, None, self.camera_angle, self.marker_odom_angle, self.body_odom_angle)
        return

if __name__ == '__main__':
    try:
        rospy.init_node('reach_the_point')
        robot = robot_stalker()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
