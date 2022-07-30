#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Point, Vector3, Transform
from tf.msg import tfMessage
from extended_object_detection.msg import SimpleObjectArray, SimpleObject

class Modifier(object):
    def __init__(self):
        self.simple_obj_sub = rospy.Subscriber('extended_object_detection/simple_objects', SimpleObjectArray, self.simple_obj_sub_cb)
        
        self.modified_msg = SimpleObjectArray()
        self.dest_pub = rospy.Publisher('simple_objects_true', SimpleObjectArray, queue_size=1)
        
        self.marker_size = rospy.get_param('~marker_size', 0.4)
        self.hori_fov = rospy.get_param('~horizontal_fov', 1.396264)
        self.width_px = rospy.get_param('~width', 640)
        self.height_px = rospy.get_param('~height', 480)
        self.focus_px = self.width_px / (2 * math.tan(self.hori_fov / 2))
        self.vert_fov = 2 * math.atan2(self.height_px, 2 * self.focus_px)
        
    def modify_msg(self):
        for i in self.modified_msg.objects:
            if i.type_id == 44:
                y1_px = i.rect.left_bottom.y
                y2_px = i.rect.right_up.y
                dy_px = y2_px - y1_px
                
                real_dist = self.marker_size * self.focus_px / dy_px
                #rospy.logwarn("m_size: {:.3f}; focus_px: {:.3f}; height_px: {:.3f}".format(self.marker_size, self.focus_px, dy_px))
                #rospy.logwarn("real_dist: {:.3f}".format(real_dist))
                
                k = real_dist / math.hypot(i.transform.translation.x, i.transform.translation.y, i.transform.translation.z)
                
                #rospy.logwarn("before: {:.3f}; after: {:.3f}".format( math.hypot(i.transform.translation.x, i.transform.translation.y, i.transform.translation.z),real_dist))
                
                i.transform.translation.x = i.transform.translation.x * k
                i.transform.translation.y = i.transform.translation.y * k
                i.transform.translation.z = i.transform.translation.z * k
                
                for j in [0, 1, 2, 3]:
                    i.rect.cornerTranslates[j].x = i.rect.cornerTranslates[j].x * k
                    i.rect.cornerTranslates[j].y = i.rect.cornerTranslates[j].y * k
                    i.rect.cornerTranslates[j].z = i.rect.cornerTranslates[j].z * k
                    
                    i.contour.contourTranslates[j].x = i.contour.contourTranslates[j].x * k
                    i.contour.contourTranslates[j].y = i.contour.contourTranslates[j].y * k
                    i.contour.contourTranslates[j].z = i.contour.contourTranslates[j].z * k
        return
        
    def simple_obj_sub_cb(self, msg):
        self.modified_msg = msg
        self.modify_msg()
        self.dest_pub.publish(self.modified_msg)
        
        return
    
if __name__ == '__main__':
    try:
        rospy.init_node('dist_modifier')
        modif = Modifier()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
    
