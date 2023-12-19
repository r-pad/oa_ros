#!/usr/bin/python3

import rospy
import math
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
import sys
from scipy.spatial.transform import Rotation as R
import numpy as np

def init_scene(scene: moveit_commander.PlanningSceneInterface):

    scene.remove_world_object("table")
    scene.remove_world_object("cam0_beam")
    scene.remove_world_object("cam1_beam")
    scene.remove_world_object("cam2_beam")
    scene.remove_world_object("cam3_beam")


    rospy.sleep(2)
    timestamp = rospy.get_rostime()

    # The table is actually a few millimeters below the world frame.
    OFFSET = -0.00635
    table_dx, table_dy, table_dz = 1.3, 1.3, 1.2
    table_x = 0.5#.4
    table_y = 0#.65
    table_z = -table_dz / 2.0 + OFFSET
    table_size = [table_dx, table_dy, table_dz]
    table_pose = PoseStamped()
    table_pose.header.frame_id = "/world"
    table_pose.header.stamp = timestamp
    table_pose.pose.orientation.w = 1
    table_pose.pose.position.x = table_x
    table_pose.pose.position.y = table_y
    table_pose.pose.position.z = table_z

    table_edge_xp = table_x + table_dx / 2.0
    table_edge_xn = table_x - table_dx / 2.0
    table_edge_yp = table_y + table_dy / 2.0
    table_edge_yn = table_y - table_dy / 2.0

    cam0_beam_pose = PoseStamped()
    cam_beam_dx, cam_beam_dy, cam_beam_dz = 0.20, 0.05, 1.219
    cam0_beam_size = [cam_beam_dx, cam_beam_dy*2, cam_beam_dz]
    cam0_beam_pose.header.frame_id = "/world"
    cam0_beam_pose.header.stamp = timestamp
    cam0_beam_pose.pose.orientation.w = 1
    cam0_beam_pose.pose.position.x = table_edge_xp 
    cam0_beam_pose.pose.position.y = table_edge_yp- 0.3
    cam0_beam_pose.pose.position.z = cam_beam_dz / 2.0 - 0.1778

    cam1_beam_pose = PoseStamped()
    cam1_beam_size = [0.05, 0.3*2, cam_beam_dz] # flip x and y
    cam1_beam_pose.header.frame_id = "/world"
    cam1_beam_pose.header.stamp = timestamp
    cam1_beam_pose.pose.orientation.w = 1
    cam1_beam_pose.pose.position.x = table_edge_xp- 0.75
    cam1_beam_pose.pose.position.y = table_edge_yp 
    cam1_beam_pose.pose.position.z = cam_beam_dz / 2.0 - 0.1778

    cam2_beam_pose = PoseStamped()
    cam2_beam_size = [0.6*2, 0.05, cam_beam_dz] 
    cam2_beam_pose.header.frame_id = "/world"
    cam2_beam_pose.header.stamp = timestamp
    cam2_beam_pose.pose.orientation.w = 1
    cam2_beam_pose.pose.position.x = table_edge_xp 
    cam2_beam_pose.pose.position.y = table_edge_yp - 1.1
    cam2_beam_pose.pose.position.z = cam_beam_dz / 2.0 - 0.1778
    
    rospy.sleep(0.5)
    scene.add_box("table", table_pose, table_size)
    scene.add_box("cam0_beam", cam0_beam_pose, cam0_beam_size)
    scene.add_box("cam1_beam", cam1_beam_pose, cam1_beam_size)
    scene.add_box("cam2_beam", cam2_beam_pose, cam2_beam_size)


def main():
    rospy.init_node("init_scene")

    scene = moveit_commander.PlanningSceneInterface()
    init_scene(scene)

    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("we don't want to continue")


if __name__ == "__main__":
    main()
