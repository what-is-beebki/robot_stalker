#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Vector3, Transform, TransformStamped
from tf.msg import tfMessage

#вытаскивает из tf координаты маркера в odom

class Filter(object):
    def __init__(self):
                
        self.tf_sub = rospy.Subscriber('/tf', tfMessage, self.cb_for_tf_sub)
        
        self.marker_trans = Transform() #позиция и ориентация маркера в odom
        
        self.dest_pub = rospy.Publisher('destination', Transform, queue_size=1)
        self.tf_prefix = rospy.get_param('~tf_prefix', '')

        
    def cb_for_tf_sub(self, msg):
        for obj in msg.transforms:
            if obj.header.frame_id == 'odom' and obj.child_frame_id == self.tf_prefix + "/" + "Aruco4_0":
                #правильнее было бы прикрутить какую-нибудь штуку, сравнивающую строки, т. к. индекс '0' может меняться
                #self.temp_pub.publish(obj)
                self.marker_trans = obj.transform
                self.dest_pub.publish(self.marker_trans)
        return
    
if __name__ == '__main__':
    try:
        rospy.init_node('estimate_dest')
        filterfilter = Filter()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
    
