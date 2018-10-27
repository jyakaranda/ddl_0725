#include <ethernet_driver/ethernet_driver.h>
#include <iostream>
#include "gtest/gtest.h"

namespace ethernet_driver
{
EthernetDriver::EthernetDriver(ros::NodeHandle &n, ros::NodeHandle &np) : n_(n), np_(np), socket_fd(-1)
{
	return;
}

EthernetDriver::~EthernetDriver()
{
	(void)close(socket_fd);
	return;
}

bool EthernetDriver::initialize()
{
	ROS_INFO("init start...");
	if (!loadParams())
	{
		ROS_ERROR("Cannot load all required ROS parameters...");
		return false;
	}

	if (!createRosIO())
	{
		ROS_ERROR("Cannot create all ROS IO...");
		return false;
	}

	if (!openUDPPort())
	{
		ROS_ERROR("Cannot open UDP port...");
		return false;
	}
	ROS_INFO("Initialised ethernet_node without error");
	ROS_INFO("init successfully end...");
	return true;
}

bool EthernetDriver::loadParams()
{
	np_.param("device_ip", device_ip_string, std::string("192.168.1.102"));
	np_.param("device_port", UDP_PORT_NUMBER, 1024);
	np_.param("debug", DEBUG, false);
	np_.param("mcu_ip", MCU_ip_string, std::string("0.0.0.0"));
	np_.param("mcu_port", MCU_UDP_PORT_NUMBER, 1025);
	np_.param<double>("wheel_radius", param_wheel_radius, 0.08);
	np_.param<double>("angle_size", param_angle_size, 0.2);
	if (!np_.getParam("pwm2rpm", param_pwm2rpm_string))
	{
		ROS_ERROR("didnot find correlation between pwm and rpm!");
		return false;
	}
	else
	{
		if (!arrayParser(param_pwm2rpm_string, param_pwm2rpm))
		{
			return false;
		}
	}
	if (!np_.getParam("pwm2angle", param_pwm2angle_string))
	{
		ROS_ERROR("didnot find correlation between pwm and angle!");
		return false;
	}
	else
	{
		if (!arrayParser(param_pwm2angle_string, param_pwm2angle))
		{
			return false;
		}
	}

	inet_aton(device_ip_string.c_str(), &device_ip);
	inet_aton(MCU_ip_string.c_str(), &MCU_ip);
	socket_fd = -1;
	ROS_INFO_STREAM("Opening UDP socket: address " << device_ip_string);
	ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
	return true;
}

bool EthernetDriver::createRosIO()
{
	diagnostics.setHardwareID("controller");

	const double diag_freq = 16 * 20000.0 / (12 * 32);
	diag_max_freq = diag_freq;
	diag_min_freq = diag_freq;
	ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

	using namespace diagnostic_updater;
	diag_topic.reset(new TopicDiagnostic(
			"lslidar_packets", diagnostics,
			FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
			TimeStampStatusParam()));

	// Output
	// packet_pub = nh.advertise<lslidar_c16_msgs::LslidarC16Packet>(
	//             "lslidar_packet", 100);
	pub_imu = n_.advertise<sensor_msgs::Imu>("/imu/data_raw", 50);
	pub_pose = n_.advertise<geometry_msgs::TwistStamped>("/raw_vel", 10);
	pub_cmd_vel = n_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/cmd_vel_fd", 10);
	sub_vel = n_.subscribe<geometry_msgs::Twist>("/cmd_vel", 50, boost::bind(&EthernetDriver::cmdCB, this, _1));

	return true;
}

bool EthernetDriver::openUDPPort()
{
	socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
	if (socket_fd == -1)
	{
		perror("socket");
		return false;
	}
	memset(&mcu_addr, 0, sizeof(mcu_addr));
	mcu_addr.sin_family = AF_INET;
	mcu_addr.sin_addr.s_addr = inet_addr(MCU_ip_string.c_str());
	mcu_addr.sin_port = htons(MCU_UDP_PORT_NUMBER);
	mcu_addr_len = sizeof(mcu_addr);

	sockaddr_in my_addr;											 // my address information
	memset(&my_addr, 0, sizeof(my_addr));			 // initialize to zeros
	my_addr.sin_family = AF_INET;							 // host byte order
	my_addr.sin_port = htons(UDP_PORT_NUMBER); // short, in network byte order
	ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
	my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill in my IP

	if (bind(socket_fd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
	{
		perror("bind"); // TODO: ROS_ERROR errno
		return false;
	}

	if (fcntl(socket_fd, F_SETFL, O_NONBLOCK | FASYNC) < 0)
	{
		perror("non-block");
		return false;
	}

	return true;
}

int EthernetDriver::getPacket(PacketData &packet, double &stamp)
{
	double time1 = ros::Time::now().toSec();

	struct pollfd fds[1];
	fds[0].fd = socket_fd;
	fds[0].events = POLLIN;
	static const int POLL_TIMEOUT = 2000; // one second (in msec)

	sockaddr_in sender_address;
	socklen_t sender_address_len = sizeof(sender_address);

	while (true)
	{
		// Unfortunately, the Linux kernel recvfrom() implementation
		// uses a non-interruptible sleep() when waiting for data,
		// which would cause this method to hang if the device is not
		// providing data.  We poll() the device first to make sure
		// the recvfrom() will not block.
		//
		// Note, however, that there is a known Linux kernel bug:
		//
		//   Under Linux, select() may report a socket file descriptor
		//   as "ready for reading", while nevertheless a subsequent
		//   read blocks.  This could for example happen when data has
		//   arrived but upon examination has wrong checksum and is
		//   discarded.  There may be other circumstances in which a
		//   file descriptor is spuriously reported as ready.  Thus it
		//   may be safer to use O_NONBLOCK on sockets that should not
		//   block.

		// poll() until input available
		do
		{
			int retval = poll(fds, 1, POLL_TIMEOUT);
			if (retval < 0) // poll() error?
			{
				if (errno != EINTR)
					ROS_ERROR("poll() error: %s", strerror(errno));
				return 1;
			}
			if (retval == 0) // poll() timeout?
			{
				ROS_WARN("mcu poll() timeout");
				return 1;
			}
			if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
			{
				ROS_ERROR("poll() reports mcu error");
				return 1;
			}
		} while ((fds[0].revents & POLLIN) == 0);

		// Receive packets that should now be available from the
		// socket using a blocking read.
		char buf[PACKET_SIZE + 1];
		ssize_t nbytes = recvfrom(socket_fd, &buf, PACKET_SIZE, 0,
															(sockaddr *)&sender_address, &sender_address_len);

		//        ROS_DEBUG_STREAM("incomplete lslidar packet read: "
		//                         << nbytes << " bytes");
		PacketData *tmp = (PacketData *)(&(buf[0]));
		packet = *tmp;
		if (nbytes < 0)
		{
			if (errno != EWOULDBLOCK)
			{
				perror("recvfail");
				ROS_INFO("recvfail");
				return 1;
			}
		}
		else if ((size_t)nbytes == PACKET_SIZE)
		{
			// read successful,
			// if packet is not from the lidar scanner we selected by IP,
			// continue otherwise we are done
			ROS_INFO("sender_address: %s, %f", inet_ntoa(sender_address.sin_addr), packet.dat.vel.liner[1]);
			if (MCU_ip_string != "" && sender_address.sin_addr.s_addr != MCU_ip.s_addr)
				continue;
			else
			{
				ssize_t bytes = sendto(socket_fd, (uint8_t *)&packet, PACKET_SIZE, 0, (struct sockaddr *)&mcu_addr, mcu_addr_len);
				ROS_INFO("send %d bytes to mcu: %s", (size_t)bytes, inet_ntoa(mcu_addr.sin_addr));
				break; //done
			}
		}
	}
	// Average the times at which we begin and end reading.  Use that to
	// estimate when the scan occurred.
	double time2 = ros::Time::now().toSec();
	stamp = ros::Time((time2 + time1) / 2.0).toSec();

	return 0;
}

bool EthernetDriver::pubMsg(PacketData data, double stamp)
{
	if (data.type == VAL_VEL)
	{
		pub_msg_pose.header.stamp = ros::Time(stamp);
		pub_msg_pose.twist.linear.x = data.dat.vel.liner[0];
		pub_msg_pose.twist.linear.y = data.dat.vel.liner[1];
		pub_msg_pose.twist.linear.z = 0;
		pub_pose.publish(pub_msg_pose);
		if (DEBUG)
		{
			ROS_INFO("VAL_POINT[%.1f,%.1f,%.1f]", pub_msg_pose.twist.linear.x, pub_msg_pose.twist.linear.y, pub_msg_pose.twist.linear.z);
		}
	}

	if (data.type == VAL_IMU)
	{
		pub_msg_imu.header.stamp = ros::Time(stamp);
		pub_msg_imu.header.frame_id = "imu_link";

		pub_msg_imu.angular_velocity.x = data.dat.vel.angular[0];
		pub_msg_imu.angular_velocity.y = data.dat.vel.angular[1];
		pub_msg_imu.angular_velocity.z = data.dat.vel.angular[2];
		pub_imu.publish(pub_msg_imu);
		if (DEBUG)
		{
			ROS_INFO("IMU RPY[%f, %f, %f]XYZ", data.dat.vel.angular[0], data.dat.vel.angular[1], data.dat.vel.angular[2]);
		}
	}
}

bool EthernetDriver::polling()
{

	return true; // 直接返回，目前没有需要从 MCU 中接收数据

	PacketData packet;
	double stamp;
	while (true)
	{
		int rc = getPacket(packet, stamp);
		// int rc = 0;
		ROS_INFO("rc: %d", rc);
		if (rc == 0)
			break;
		if (rc < 0)
			return false;
	}

	ROS_DEBUG("receive a packet");

	if (stamp > 0)
	{
		pubMsg(packet, stamp);
	}

	// diag_topic->tick(ros::Time(stamp));
	// diagnostics.update();

	return true;
}

// 接收 TX2 发出的控制信息，通过以太网发送给 MCU 。
void EthernetDriver::cmdCB(const geometry_msgs::TwistConstPtr &msg)
{
	if (socket_fd < 0)
	{
		ROS_ERROR("Socket didnot successfully bind!");
		return;
	}
	double pwm1 = 15400.;
	double rpm = msg->linear.x * K4RPM / param_wheel_radius;
	int idx = findPwmIdx(param_pwm2rpm, rpm);
	if (idx < 0)
	{
		idx = 0;
	} else if (idx == 0 && rpm > 50){
		// 如果速度大于 0.4，则前进
		idx = 1;
	}
	// ROS_INFO("idx: %d, rpm: %.1f", idx, rpm);

	// 计算转角对应的 pwm 波，存在一定的四舍五入，而且需要注意 param_pwm2angle 中的值不完全成比例
	double pwm2 = 15600.;
	int l = 0;
	double angle = msg->angular.z / PI * 180;
	if (angle <= param_pwm2angle[0].y)
	{
		pwm2 = param_pwm2angle[0].x;
	}
	else if (angle >= param_pwm2angle[param_pwm2angle.size() - 1].y)
	{
		pwm2 = param_pwm2angle[param_pwm2angle.size() - 1].x;
	}
	else
	{
		for (; l < param_pwm2angle.size() - 1; l++)
		{
			if (angle > param_pwm2angle[l].y && angle <= param_pwm2angle[l + 1].y)
			{
				break;
			}
		}
		pwm2 = param_pwm2angle[l].x + (angle - param_pwm2angle[l].y) / (param_pwm2angle[l + 1].y - param_pwm2angle[l].y) * (param_pwm2angle[l + 1].x - param_pwm2angle[l].x);
		pwm2 = floor(pwm2 / 10) * 10 + ((int)pwm2 % 10 >= 5 ? 10 : 0);
	}
	static PacketData packet_pub;
	packet_pub.type = VAL_VEL;
	packet_pub.syn = 0xFA;
	packet_pub.dat.vel.liner[0] = param_pwm2rpm[idx].x;
	packet_pub.dat.vel.liner[1] = msg->linear.y;
	packet_pub.dat.vel.liner[2] = msg->linear.z;
	packet_pub.dat.vel.angular[0] = msg->angular.x;
	packet_pub.dat.vel.angular[1] = msg->angular.y;
	packet_pub.dat.vel.angular[2] = pwm2;
	packet_pub.syn_CR = 100;
	packet_pub.syn_LF = '\n';

	ROS_INFO("send packet to mcu, linear.x: %.4f ; angular.z: %.4f \n rpm: %.1f ; angle: %.1f", msg->linear.x, angle, packet_pub.dat.vel.liner[0], packet_pub.dat.vel.angular[2]);

	ssize_t nbytes = sendto(socket_fd, (uint8_t *)&packet_pub, PACKET_SIZE, 0, (struct sockaddr *)&mcu_addr, mcu_addr_len);
	if ((size_t)nbytes != PACKET_SIZE)
	{
		ROS_ERROR("cannot send cmd_vel to MCU!");
	}

	// 小车实际的运行速度和转角，存在一定的误差
	static geometry_msgs::TwistWithCovarianceStamped vel_fd;
	vel_fd.header.frame_id = std::string("/base_link");
	vel_fd.header.stamp = ros::Time::now();
	vel_fd.twist.twist.linear.x = param_pwm2rpm[idx].y * param_wheel_radius / K4RPM;
	vel_fd.twist.twist.angular.z = (param_pwm2angle[l].y + (angle - param_pwm2angle[l].y) / (param_pwm2angle[l + 1].y - param_pwm2angle[l].y) * (param_pwm2angle[l + 1].y - param_pwm2angle[l].y)) * PI / 180;
	// TODO: twist.covariance 误差还需要添加进去
	pub_cmd_vel.publish(vel_fd);
}

bool EthernetDriver::arrayParser(const std::string &s_in, std::vector<geometry_msgs::Point32> &v_out)
{
	using namespace std;
	v_out.clear();
	stringstream input_ss(s_in);
	int depth = 0;
	geometry_msgs::Point32 point;
	vector<vector<double> > vec;
	vector<double> current_vec;
	while (!!input_ss && !input_ss.eof())
	{
		switch (input_ss.peek())
		{
		case EOF:
			break;
		case '[':
			depth++;
			if (depth > 2)
			{
				v_out.clear();
				ROS_ERROR("array depth greater than 2");
				return false;
			}
			input_ss.get();
			current_vec.clear();
			break;
		case ']':
			depth--;
			if (depth < 0)
			{
				v_out.clear();
				ROS_ERROR("syntax error: more close ] than open [");
				return false;
			}
			input_ss.get();
			if (depth == 1)
			{
				vec.push_back(current_vec);
			}
			break;
		case ',':
		case ' ':
		case '\t':
		case '\n':
		case '\r':
			input_ss.get();
			break;
		default:
			if (depth != 2)
			{
				v_out.clear();
				ROS_ERROR("syntax error: num at depth other than 2. char was %c", char(input_ss.peek()));
				return false;
			}
			double value;
			input_ss >> value;
			if (!!input_ss)
			{
				current_vec.push_back(value);
			}
			break;
		}
	}

	if (depth != 0)
	{
		v_out.clear();
		ROS_ERROR("syntax error: unterminated vector string");
		return false;
	}

	for (int i = 0; i < vec.size(); i++)
	{
		if (vec[i].size() != 2)
		{
			v_out.clear();
			ROS_ERROR("points in the pwm2rpm/pwm2angle specification must be pairs of numbers, found a point with %d numbers!", vec[i].size());
			return false;
		}
		else
		{
			point.x = vec[i][0];
			point.y = vec[i][1];
			v_out.push_back(point);
		}
	}
	return true;
}

} // namespace ethernet_driver
