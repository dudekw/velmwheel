#!/usr/bin/env python
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import xml.etree.ElementTree
import string
import sys

def talker(file_name):
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)
    rospy.init_node('show_map', anonymous=True)
    root = xml.etree.ElementTree.parse(file_name).getroot()

    my_marker = Marker()
    my_marker.header.seq = 0
    my_marker.header.stamp = rospy.Time.now()
    my_marker.header.frame_id = "odom"

    my_marker.ns = "show_map"
    my_marker.id = 1
    my_marker.type = 8
    my_marker.action = 0
    my_marker.lifetime = rospy.Duration(0.5)
    my_marker.scale.x = 0.1
    my_marker.scale.y = 0.1
    my_marker.frame_locked = True
    my_point = Point()
    my_point.z = 0 
    color = ColorRGBA()
    color.g = 0
    color.b = 0
    color.a = 1
    color.r = 1
    merker_size = len(root.getchildren())
    for i in range(len(root.getchildren())):
        my_point.x = float(root[i][0].text)
        my_point.y = float(root[i][1].text)
        my_marker.points.insert(i,Point(float(root[i][0].text), float(root[i][1].text), 0))
        my_marker.colors.insert(i,color)
        print i
        #i = i + 1
    

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(my_marker)

#        hello_str = "hello world %s" % rospy.get_time()
#        rospy.loginfo(hello_str)
#        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        file = sys.argv[1]
        talker(file)
    except rospy.ROSInterruptException:
        pass