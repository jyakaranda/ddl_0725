# #!/usr/bin/env python

import pcl_ros
import rosbag
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2


def main():
    rospy.init_node('pc2scan', anonymous=True)
    print 'from pointcloud to laserscan'
    filename = '/home/zh/workspace/catkin_ws/src/bag_file/0802_1000_all.bag'
    count = 0
    with rosbag.Bag(filename, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/lslidar_point_cloud':
                return


if __name__ == '__main__':
    main()
