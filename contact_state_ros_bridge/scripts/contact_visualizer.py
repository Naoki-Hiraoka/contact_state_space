#!/usr/bin/env python

import numpy
import rospy

from contact_state_msgs.msg import ContactArray

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import time

pub_marker = None
sub_contact = None

def callback(msg) :
    # reduce CPU load
    if pub_marker.get_num_connections() == 0:
        return

    now = rospy.Time.now()
    markerArray = MarkerArray()
    for contact in msg.contacts:
        sphere_color = ColorRGBA(0,1,0,0.5)
        sphere_scale = 0.02

        marker = Marker()
        marker.header.frame_id = contact.pose.header.frame_id
        marker.header.stamp = now
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale = Vector3(sphere_scale, sphere_scale, sphere_scale)
        marker.color = sphere_color
        marker.pose = contact.pose.pose
        markerArray.markers.append(marker)

    id = 0
    for m in markerArray.markers:
        m.lifetime = rospy.Duration(0.1)
        m.id = id
        id += 1

    pub_marker.publish(markerArray)


if __name__ == '__main__':
    rospy.init_node("contact_visualizer")

    pub_marker = rospy.Publisher('~marker', MarkerArray, queue_size=1)
    sub_contact = rospy.Subscriber("~contact", ContactArray, callback, queue_size=1, buff_size=10000000) # buff_size is necessary to avoid lag

    rospy.spin()
