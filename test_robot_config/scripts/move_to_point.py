#!/usr/bin/env python3
import math

import rospy

from geometry_msgs.msg import Twist,Point
from gazebo_msgs.msg import LinkStates

def angle_diff(a1, a2):
    diff = a1 - a2
    return (diff + math.pi) % (2*math.pi) - math.pi

class MoveToPointController(object):
    def __init__(self):
        
        self.robot_link = rospy.get_param('~robot_link', 'test_robot::base_link')
        self.x = 0
        self.y = 0
        self.angle = 0
        self.goal_x = None
        self.goal_y = None
        
        self.velocity_msg = Twist()
        self.velocity_pub = rospy.Publisher('mobile_base_controller/cmd_vel',
                                            Twist,
                                            queue_size=1)
        self.cmd_sub = rospy.Subscriber('cmd_pos', Point, self.cb_command)
        self.joints_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.cb_link_states)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_timer)
        
    def cb_command(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        
        
    def cb_timer(self, e):
        if self.goal_x is None:
            return
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        goal_angle = math.atan2(dy, dx)
        diff = angle_diff(goal_angle, self.angle)
        rospy.logerr('ori: {}, pos:{} {}, diff: {:.3f}, dist: {:.3f}, g: {} {}'
                     .format(self.angle, self.x, self.y, diff * 180 / math.pi, math.hypot(dx, dy), self.goal_x, self.goal_y))
        
        if math.hypot(dx, dy)<0.02:
            self.goal_x = None
            return
        if abs(diff) > 0.02:
            self.velocity_msg.angular.z = 0.5 if diff > 0 else -0.5
            self.velocity_msg.linear.x = 0
        else:
            self.velocity_msg.angular.z = 0
            self.velocity_msg.linear.x = 0.5
        self.velocity_pub.publish(self.velocity_msg)
            
        
        
        
    def cb_link_states(self, msg):
        try:
            robot_ind = msg.name.index(self.robot_link)
        except:
            return
        pos = msg.pose[robot_ind].position
        ori = msg.pose[robot_ind].orientation
        self.x = pos.x
        self.y = pos.y
        self.angle = 2 * math.atan2(ori.z, ori.w)
        #self.skip +=1
        #if self.skip %100 == 0:
        #    rospy.logerr('LS: {:.3f} {:.3f} {:.3f}'.format(x, y, angle))

if __name__ == '__main__':
    try:
        rospy.init_node('move_to_point')
        mc = MoveToPointController()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
