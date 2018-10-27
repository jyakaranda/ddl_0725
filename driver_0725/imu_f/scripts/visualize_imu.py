import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt

x = []
y1 = []
y2 = []
y3 = []


def callback(imu_msg):
    fig = plt.figure()
    fig_1 = fig.add_subplot(2,2,1)
    fig_2 = fig.add_subplot(2,2,2)
    fig_3 = fig.add_subplot(2,2,3)
    x.append(len(x))
    y1.append(imu_msg.linear_acceleration.x)
    y2.append(imu_msg.linear_acceleration.y)
    y3.append(imu_msg.angular_velocity.z)
    p1 = plt.scatter(x, y1,  c='r', marker='.')
    p2 = plt.scatter(x, y2,  c='b', marker='.')
    p3 = plt.scatter(x, y3,  c='g', marker='.')

    p1.set_title('acc_x')
    p2.set_title('acc_y')
    p3.set_title('ang_z')

    plt.show()
    plt.pause(0.05)


def main():
    rospy.init_node("visualize_node")
    print 'Start visualize_node.'
    sub_wheel = rospy.Subscriber('/imu/data', Imu, callback)
    plt.ion()
    rospy.spin()


if __name__ == '__main__':
main()