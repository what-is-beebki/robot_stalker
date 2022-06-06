#!/usr/bin/env python3
import math
import rospy

from geometry_msgs.msg import Twist, Point, Vector3
from std_msgs.msg import Int32, Float64

from extended_object_detection.msg import SimpleObject, SimpleObjectArray


class Filter(object):
    def __init__(self):
                
        self.the_very_id = 44
        self.the_very_number = '13'
        self.real_size = 0.4 #вообще-то нет
        
        self.object_array_sub = rospy.Subscriber('/extended_object_detection/simple_objects', SimpleObjectArray, self.cb_object_array)
        
        self.destination_msg = Vector3()
        self.destination_pub = rospy.Publisher('/destination', Point, queue_size=1)
        #self.objest_msg = SimpleObject()
        #self.object_pub = rospy.Publisher('/simp_obj', SimpleObject, queue_size=1)
        
    def cb_object_array(self, msg):
        for obj in msg.objects:
            if obj.type_id == self.the_very_id:
                #может, выкинуть первое условие? по-хорошему, id однозначно задаёт ключ
                if obj.extracted_info.keys[0] == 'Aruco4:marker_id' and obj.extracted_info.values[0] == self.the_very_number:
                    #self.objest_msg = obj
                    #self.object_pub.publish(self.objest_msg)
                    self.dist = self.calc_dist(obj.contour.contourTranslates)
                    self.build_msg(obj.transform.translation)
                    self.destination_pub.publish(self.destination_msg)
                    return
        return
    def build_msg(self, tl_list):
        self.destination_msg.x = tl_list.z
        self.destination_msg.y = - tl_list.x
        self.destination_msg.z = - tl_list.y
        
        #self.destination_msg.x = tl_list.z * self.dist
        #self.destination_msg.y = - tl_list.x * self.dist
        #self.destination_msg.z = - tl_list.y * self.dist
    def calc_dist(self, tl_list):
        point1_x = (tl_list[0].x + tl_list[1].x) / 2
        point1_y = (tl_list[0].y + tl_list[1].y) / 2
        point2_x = (tl_list[2].x + tl_list[3].x) / 2
        point2_y = (tl_list[2].y + tl_list[3].y) / 2
        rel_size = math.hypot(point1_x - point2_x, point1_y - point2_y)
        return self.real_size / rel_size
    
if __name__ == '__main__':
    try:
        rospy.init_node('detect_the_marker')
        filterfilter = Filter()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        pass
    
