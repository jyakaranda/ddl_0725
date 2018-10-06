#include "imu_vel.h"

ImuVel::ImuVel(ros::NodeHandle nh, ros::NodeHandle pnh) : //  Members default values
                                                          nh_(nh),
                                                          pnh_(pnh)
{
    ROS_INFO("ImuVel aleady");
    sub = nh_.subscribe("/imu/data", 500, &ImuVel::vel_callback, this);
    pub = nh_.advertise<geometry_msgs::TwistStamped>("/imu/twist", 500);
    
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
    
    double acceleration_x = imu_msg->linear_acceleration.x;
    double acceleration_y = imu_msg->linear_acceleration.y;

    if (acceleration_x < 0.01 && acceleration_x > -0.01)
        acceleration_x = 0;
    if (acceleration_y < 0.01 && acceleration_y > -0.01)
        acceleration_y = 0;

    double delta_x = acceleration_x * (end - begin).toSec();
    double delta_y = acceleration_y * (end - begin).toSec();

    geometry_msgs::TwistStampedPtr vel_msg = boost::make_shared<geometry_msgs::TwistStamped>();
    vel_x += delta_x;
    vel_y += delta_y;

    vel_msg->twist.linear.x = vel_x;
    vel_msg->twist.linear.y = vel_y;
    vel_msg->header.stamp = end;
    vel_msg->twist.angular.z = 0;

    ROS_INFO("acc_Liner[x=%f,y=%f]", imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y);
    ROS_INFO("Liner[x=%f,y=%f]", vel_msg->twist.linear.x, vel_msg->twist.linear.y);
    pub.publish(vel_msg);
    begin = end;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_vel");
    ros::NodeHandle nh,pnh("~");
    ImuVel ImuVel(nh, pnh);

    ros::spin();
    return 0;
}
