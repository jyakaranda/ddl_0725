#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
import matplotlib.pyplot as plt

x = []
y1 = []
y2 = []


def wheelCB(msg):
    x.append(len(x))
    y1.append(msg.twist.linear.x)
    y2.append(msg.twist.linear.y)
    p1 = plt.scatter(x, y2, s=75, c='r', marker='.', alpha=0.5)
    p2, = plt.plot(x, y1, lw=2, color='b')
    plt.legend([p1, p2], ['Measurements', 'Kalman filter'])
    # if msg.header.seq % 5 == 0:
    plt.show()
    plt.pause(0.05)
    # if msg.header.seq % 100 == 0:
    #   x[:] = []
    #   y1[:] = []
    #   y2[:] = []


def hallCB(msg):
    x.append(len(x))
    y1.append(msg.data)
    p1 = plt.scatter(x, y1, s=75, c='r', marker='.', alpha=0.5)
    plt.legend([p1], ['Hall'])
    if len(x) % 60 == 0:
        plt.show()
        plt.pause(0.01)
    # if msg.header.seq % 100 == 0:
    #   x[:] = []
    #   y1[:] = []
    #   y2[:] = []

''' Visualize raw_vel and filtered_vel to comparing.
'''
def main():
    rospy.init_node("visualize_node")
    print 'Start visualize_node.'
    sub_wheel = rospy.Subscriber('/wheel_circles', TwistStamped, wheelCB)
    # sub_hall = rospy.Subscriber('/hall_sensor', Bool, hallCB)
    plt.ion()
    rospy.spin()


if __name__ == '__main__':
    main()
