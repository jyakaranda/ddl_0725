#include "imu_vel.h"

ImuVel::ImuVel(ros::NodeHandle nh, ros::NodeHandle pnh) : //  Members default values
                                                          nh_(nh),
                                                          pnh_(pnh)
{
    ROS_INFO("ImuVel aleady");
    sub = nh_.subscribe("/imu/data", 50, &ImuVel::vel_callback, this);
    pub = nh_.advertise<geometry_msgs::Twist>("/imu", 50);
}

void ImuVel::vel_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (!flag_first)
    {
        end = imu_msg->header.stamp;
        begin = end;
        flag_first = !flag_first;
    }
    end = imu_msg->header.stamp;

    //ROS_INFO("time=%f", end.toSec()-begin.toSec());

    double acceleration_x = imu_msg->linear_acceleration.x;
    double acceleration_y = imu_msg->linear_acceleration.y;

    if (acceleration_x < 0.01 && acceleration_x > -0.01)
        acceleration_x = 0;
    if (acceleration_y < 0.01 && acceleration_y > -0.01)
        acceleration_y = 0;

    double delta_x = acceleration_x * (end - begin).toSec();
    double delta_y = acceleration_y * (end - begin).toSec();

    geometry_msgs::TwistPtr vel_msg = boost::make_shared<geometry_msgs::Twist>();
    vel_x += delta_x;
    vel_y += delta_y;

    vel_msg->linear.x = vel_x;
    vel_msg->linear.y = vel_y;
    vel_msg->linear.y = 0;
    vel_msg->angular.z = imu_msg->angular_velocity.z;
    pub.publish(vel_msg);
    begin = end;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_vel");
    ros::NodeHandle nh, pnh("~");
    ImuVel ImuVel(nh, pnh);

    ros::spin();
    return 0;
}