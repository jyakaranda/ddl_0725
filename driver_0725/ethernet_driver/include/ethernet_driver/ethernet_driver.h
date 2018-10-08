#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ethernet_driver/user_protocol.h>

#include <string>
#include <sstream>
#include <cstdio>
#include <cmath>
#include <vector>
#include <map>

namespace ethernet_driver
{
#define PI 3.1415926
#define K4RPM 9.54929675 // = 60/(2*PI) ，用于速度与转速之间的转换：rpm = (v*60)/(2*PI*r)
class EthernetDriver
{
public:
	EthernetDriver(ros::NodeHandle &n, ros::NodeHandle &np);
	~EthernetDriver();
	bool initialize();
	bool polling();

	typedef boost::shared_ptr<EthernetDriver> EthernetDriverPtr;
	typedef boost::shared_ptr<const EthernetDriver> EthernetDriverConstPtr;

private:
	bool DEBUG;

	// ROS related variables

	ros::NodeHandle n_;
	ros::NodeHandle np_;
	std::string frame_id;
	ros::Publisher packet_pub;
	ros::Publisher pub_imu;
	ros::Publisher pub_pose;
	ros::Publisher pub_cmd_vel; // 用于返回电机和舵机的实际运行速度，误差存储在 covariance 中
	geometry_msgs::TwistStamped pub_msg_pose;
	sensor_msgs::Imu pub_msg_imu;
	ros::Subscriber sub_vel;

	bool loadParams();
	bool createRosIO();
	bool openUDPPort();
	int getPacket(PacketData &packet, double &stamp);
	bool pubMsg(PacketData packet, double stamp);
	void cmdCB(const geometry_msgs::TwistConstPtr &msg);

	// Ethernet relate variables
	std::string device_ip_string;
	in_addr device_ip;
	int UDP_PORT_NUMBER;
	int socket_fd;
	struct sockaddr_in mcu_addr;
	socklen_t mcu_addr_len;
	std::string MCU_ip_string;
	in_addr MCU_ip;
	int MCU_UDP_PORT_NUMBER;
	// int MCU_socket_fd;
	double param_wheel_radius;
	// point.x: pwm 波, point.y: rpm/angle, point.z: 0
	std::vector<geometry_msgs::Point32> param_pwm2rpm;
	std::vector<geometry_msgs::Point32> param_pwm2angle;
	std::string param_pwm2rpm_string;
	std::string param_pwm2angle_string;
	double param_angle_size;

	// Diagnostics updater
	diagnostic_updater::Updater diagnostics;
	boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
	double diag_min_freq;
	double diag_max_freq;

	// 将 yaml 中的 pwm 字符串解析为数组
	bool arrayParser(const std::string &s_in, std::vector<geometry_msgs::Point32> &v_out);
	int findPwmIdx(const std::vector<geometry_msgs::Point32> &pwm_vec, double value)
	{
		int l = 0, r = pwm_vec.size() - 1, mid = (l + r) / 2;
		while (l <= pwm_vec.size() - 1 && r >= 0 && l <= r)
		{
			mid = (l + r) / 2;
			if (value < pwm_vec[mid].y - 0.1)
			{
				r = mid - 1;
			}
			else if (value > pwm_vec[mid].y + 0.1)
			{
				l = mid + 1;
			}
			else
			{
				break;
			}
		}
		if (l >= pwm_vec.size())
		{
			return pwm_vec.size() - 1;
		}
		else if (r < 0)
		{
			return 0;
		}
		else if (l == r){
			return l;
		} else{
			return fabs(pwm_vec[l].y-value)<fabs(pwm_vec[r].y-value)?l:r;
		}
	}
};

} // namespace ethernet_driver