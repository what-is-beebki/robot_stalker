#!/usr/bin/env python3

import rospy
import math
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState
from tf.transformations import quaternion_from_euler

class ObjMover(object):
    def __init__(self):
        
        self.base_msg = LinkState()
        self.base_msg.link_name = rospy.get_param('~who_moves', "marker::base_link")
        self.base_msg.reference_frame = 'hall::floor'
        self.base_msg.pose.position.z = rospy.get_param('~height', 0.2)
        
        #параметры окружности, вдоль которой движется джентльмен
        self.center = [3, -3.]
        self.radius = 3.
        self.speed = 0.2   # скорость
        self.koef = self.speed / self.radius
                            # множитель в аргументе параметризации
        self.theta = 0      # сдвиг фазы
        
        self.start_time = rospy.Time.now()
        self.time = 0
        
        self.state_msg = None
        
        rospy.wait_for_service('/gazebo/set_link_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        
        self.timer = rospy.Timer(rospy.Duration(0.2), self.cb_timer)
        
    def cb_timer(self, e):
        self.make_msg()
        self.time = (rospy.Time.now() - self.start_time).to_sec()
        
        try:
            resp = self.set_state(self.state_msg)

        except rospy.ServiceException:
            pass
        
    def make_msg(self):
        self.state_msg = self.base_msg
        
        #параметризованная окружность
        arg = self.koef * self.time + self.theta
        self.state_msg.pose.position.x = self.center[0] + self.radius * math.sin(arg)
        self.state_msg.pose.position.y = self.center[1] + self.radius * math.cos(arg)
        
        self.state_msg.twist.linear.x = self.speed * math.cos(arg)
        self.state_msg.twist.linear.y = - self.speed * math.sin(arg)
        #эффект заметен, если смотреть в rviz
        
        qua = quaternion_from_euler(0, 0, - arg)
        self.state_msg.pose.orientation.x = qua[0]
        self.state_msg.pose.orientation.y = qua[1]
        self.state_msg.pose.orientation.z = qua[2]
        self.state_msg.pose.orientation.w = qua[3]
        
        return
        
    
if __name__ == '__main__':
    try:
        rospy.init_node('marker_moves')
        mover = ObjMover()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
    
