#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.baseWp = None
        self.wp2D = None
        self.wpTree = None
        self.pose = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.baseWp:
                idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        pos = self.pose.pose.position
        x, y = pos.x, pos.y
        closestIdx = self.wpTree.query([x, y], 1)[1]
        
        coord = self.wp2D[closestIdx]
        prev = self.wp2D[closestIdx - 1]
        
        coord = np.array(coord)
        prev = np.array(prev)
        pos = np.array([x, y])

        v = np.dot(coord - prev, pos - coord)

        return (closestIdx + 1) % len(self.wp2D) if v > 0 else closestIdx

    def publish_waypoints(self, closestIdx):
        lane = Lane()
        lane.header = self.baseWp.header
        lane.waypoints = self.baseWp.waypoints[closestIdx:closestIdx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose = msg
        pass

    def waypoints_cb(self, waypoints):
        self.baseWp = waypoints
        if not self.wp2D:
            self.wp2D = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in waypoints.waypoints]
            self.wpTree = KDTree(self.wp2D)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
