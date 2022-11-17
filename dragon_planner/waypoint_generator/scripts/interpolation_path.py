#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import yaml
import numpy as np
import pandas as pd
from scipy.interpolate import interp1d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class CubicSplinePath(Node):
    def __init__(self):
        super().__init__('cubic_spline_path')

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(
            Path, '/planning/path', qos_profile)
        waypoint_path = self.declare_parameter('waypoint_path', '/home/haze/Desktop/waypoint.csv').value
        waypoints = pd.read_csv(waypoint_path)
        #waypoint_x, waypoint_y = self.interpolation(waypoints)

        path = Path()
        for index in range(len(waypoints['pose.position.x'])):
            pose = PoseStamped()
            pose.pose.position.x = waypoints['pose.position.x'][index]
            pose.pose.position.y = waypoints['pose.position.y'][index]
            path.poses.append(pose)
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(path)

    def interpolation(self, waypoints, alg="cubic"):
        ix = iy = []

        #temp_x = temp_y = np.array([])
        temp_x = []
        temp_y = []
        for idx in range(len(waypoints)):
            temp_x.append(waypoints['pose.position.x'][idx])
            temp_y.append(waypoints['pose.position.y'][idx])
            #temp_x = np.append(temp_x, waypoints['pose.position.x'][idx])
            #temp_y = np.append(temp_y, waypoints['pose.position.y'][idx])
            print(temp_x, temp_y)
        cubic_spline = None
        if alg == "linear":
            cubic_spline = interp1d(temp_x, temp_y)
        elif alg == "cubic":
            cubic_spline = interp1d(temp_x, temp_y, kind='cubic')

        waypoint_x_start = temp_x[0]
        waypoint_x_end = temp_x[-1]
        length = (int)(abs(waypoint_x_end - waypoint_x_start)/0.01)

        ix = np.linspace(waypoint_x_start, waypoint_x_end, num=length)
        iy = cubic_spline(ix)

        return ix, iy


def main():
    rclpy.init()
    node = CubicSplinePath()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
