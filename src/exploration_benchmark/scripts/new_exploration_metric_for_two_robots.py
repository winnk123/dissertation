#! /usr/bin/env python2.7

'''
subscribe /robot1/scan_map  /robot2/scan_map  /robot1/map  /robot2/map  /robot1/odom  /robot2/odom

'''
import sys
import time
import os
import numpy as np
import rospy
from std_msgs.msg import String, Float32
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped
import tf
import pickle
import yaml
from PIL import Image
import math

exploration_rate_log = []
odom_log = []
path_length_log = []
odom_log_2 = []
path_length_log_2 = []
time_log = []
end_flag = False
achieve_90 = False
start_time = 0

gt_area = 0

num_robots = 2

single_map_list = [OccupancyGrid() for i in range(num_robots)]
single_robot_coverage_rate_list = [0 for i in range(num_robots)]

explo_status_pub = rospy.Publisher('explo_status', String, queue_size=10)

def get_gt(pgm_file, yaml_file):
    map_img = np.array(Image.open(pgm_file))
    map_info = yaml.safe_load(open(yaml_file, mode='r'))
    gt_area = (np.sum((map_img != 205).astype(int)))*map_info['resolution']*map_info['resolution']
    return gt_area

def callback_for_robot1(data):
# -1:unkown 0:free 100:obstacle
    global end_flag, start_time, achieve_90
    if not end_flag:
        gridmap = np.array(data.data).reshape((data.info.height, data.info.width))
        explored_map = (gridmap != -1).astype(int)
        explored_area = explored_map.sum()*data.info.resolution*data.info.resolution
        exploration_rate = explored_area / gt_area
        exploration_rate_over_time = dict()
        exploration_rate_over_time['time'] = data.header.stamp
        exploration_rate_over_time['rate'] = exploration_rate

        exploration_rate_log.append(exploration_rate_over_time)
        if exploration_rate >= 0.99:
            explo_status_pub.publish("Explo_end")
            print("exploration ends!")
            total_time = rospy.get_time()-start_time
            print("T_total: %f"%total_time)
            # compute coverage rsd
            coverage_rsd = 100*np.std(np.array(single_robot_coverage_rate_list))/np.mean(np.array(single_robot_coverage_rate_list))
            print("exploration coverage rsd: %f"%coverage_rsd)
            # compute overlap percentage
            overlap_rate = 100*(np.sum(np.array(single_robot_coverage_rate_list)) - 1)
            print("exploration overlap percentage: %f"%overlap_rate)
            end_flag = True        
        rospy.sleep(1)
    else:
        rospy.sleep(1)


def callback_for_robot2(data):
# -1:unkown 0:free 100:obstacle
    global end_flag, start_time, achieve_90
    if not end_flag:
        gridmap = np.array(data.data).reshape((data.info.height, data.info.width))
        explored_map = (gridmap != -1).astype(int)
        explored_area = explored_map.sum()*data.info.resolution*data.info.resolution
        exploration_rate = explored_area / gt_area
        exploration_rate_over_time = dict()
        exploration_rate_over_time['time'] = data.header.stamp
        exploration_rate_over_time['rate'] = exploration_rate

        exploration_rate_log.append(exploration_rate_over_time)
        if exploration_rate >= 0.99:
            explo_status_pub.publish("Explo_end")
            print("exploration ends!")
            total_time = rospy.get_time()-start_time
            print("T_total: %f"%total_time)
            # compute coverage rsd
            coverage_rsd = 100*np.std(np.array(single_robot_coverage_rate_list))/np.mean(np.array(single_robot_coverage_rate_list))
            print("exploration coverage rsd: %f"%coverage_rsd)
            # compute overlap percentage
            overlap_rate = 100*(np.sum(np.array(single_robot_coverage_rate_list)) - 1)
            print("exploration overlap percentage: %f"%overlap_rate)
            end_flag = True        
        rospy.sleep(1)
    else:
        rospy.sleep(1)

def odom_callback_for_robot1(data):
    global end_flag
    if not end_flag:
        current_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
        odom_over_time = dict()
        odom_over_time['time'] = data.header.stamp
        odom_over_time['odom'] = current_pos
        if len(odom_log) == 0:
            odom_log.append(odom_over_time)
            path_length_over_time = dict()
            path_length_over_time['time'] = data.header.stamp
            path_length_over_time['path_length'] = 0
            path_length_log.append(path_length_over_time)
        else:
            path_length_over_time = dict()
            path_length_over_time['time'] = data.header.stamp
            path_length_over_time['path_length'] = path_length_log[-1]['path_length'] + math.hypot(odom_log[-1]['odom'][0]-current_pos[0], odom_log[-1]['odom'][1]-current_pos[1])
            path_length_log.append(path_length_over_time)
            odom_log.append(odom_over_time)
        rospy.sleep(1)
    else:
        rospy.sleep(1)

def odom_callback_for_robot2(data):
    global end_flag
    if not end_flag:
        current_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
        odom_over_time = dict()
        odom_over_time['time'] = data.header.stamp
        odom_over_time['odom'] = current_pos
        if len(odom_log_2) == 0:
            odom_log_2.append(odom_over_time)
            path_length_over_time = dict()
            path_length_over_time['time'] = data.header.stamp
            path_length_over_time['path_length'] = 0
            path_length_log_2.append(path_length_over_time)
        else:
            path_length_over_time = dict()
            path_length_over_time['time'] = data.header.stamp
            path_length_over_time['path_length'] = path_length_log_2[-1]['path_length'] + math.hypot(odom_log_2[-1]['odom'][0]-current_pos[0], odom_log_2[-1]['odom'][1]-current_pos[1])
            path_length_log_2.append(path_length_over_time)
            odom_log_2.append(odom_over_time)
        rospy.sleep(1)
    else:
        rospy.sleep(1)

def single_map_callback(data):
    global single_map_list
    single_map_list[int(data.header.frame_id[5])-1] = data

def single_robot_coverage_rate_callback(data):
    global single_robot_coverage_rate_list, gt_area
    gridmap = np.array(data.data).reshape((data.info.height, data.info.width))
    explored_map = (gridmap != -1).astype(int)
    explored_area = explored_map.sum()*data.info.resolution*data.info.resolution
    exploration_rate = explored_area / gt_area
    single_robot_coverage_rate_list[int(data.header.frame_id[5])-1] = exploration_rate

def RvizCallback(data):
    global start_time
    start_time = rospy.get_time()
    print("Exploration start!")
    print("Start time: ", start_time)

def main():
    global gt_area, start_time
    # gt_area = get_gt(argv[1], argv[2]) 
    map_param = rospy.get_param("/evaluation/map")
    map_path = os.path.expanduser('~') + "/Khattiya-Explore-Bench/src/exploration_benchmark/blueprints/"
    gt_area = get_gt(map_path + map_param + ".pgm", map_path + map_param + ".yaml")
    rospy.init_node('exploration_metric', anonymous=True)
    rospy.Subscriber("/clicked_point", PointStamped, RvizCallback)
    rospy.Subscriber("/robot1/map", OccupancyGrid, callback_for_robot1, queue_size=1)
    rospy.Subscriber("/robot2/map", OccupancyGrid, callback_for_robot2, queue_size=1)
    rospy.Subscriber("/robot1/odom", Odometry, odom_callback_for_robot1, queue_size=1)
    rospy.Subscriber("/robot2/odom", Odometry, odom_callback_for_robot2, queue_size=1)
    for i in range(num_robots):
        rospy.Subscriber("/robot"+str(i+1)+"/cartographer_discrete_map", OccupancyGrid, single_map_callback, queue_size=1)
        rospy.Subscriber("/robot"+str(i+1)+"/cartographer_discrete_map", OccupancyGrid, single_robot_coverage_rate_callback, queue_size=1)
    while not rospy.is_shutdown():
        if start_time > 0:
            explo_status_pub.publish("Explo_start")
            break
    rospy.spin()


if __name__ == '__main__':
    main()