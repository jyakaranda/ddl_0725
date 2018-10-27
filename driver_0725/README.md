# 

## hall_sensor

> `desc`: 根据霍尔传感器计算转速，~~但目前不知道是程序问题还是什么原因，圈数的计数存在一定的波动，所以~~在程序中增加了一个 kalman filter 并对速度进行了平滑处理。

- [x] 之前计数存在波动且接上霍尔传感器后 tx2 容易死机的问题，是霍尔传感器(3.8V ~ 5.0V)与 tx2(1.8V ~ 3.3V) 电压不匹配导致的，通过示波器可以看到接上 pin16 后，霍尔传感器的方波很不稳定。 在霍尔传感器的连接处添加了一个电阻之后问题解决。
- [x] 由于之前使用的磁铁太小，在轮速较大的时候，磁铁扫过的方波就很小，速度更大的时候 gpio 的读数可能会直接跳过这些方波，因为 ros_gpio 中设置的 poll 时长为 100ms (影响不大)，频率为 60Hz (约 sleep 16.7ms，存在一定影响)。 将多个小磁铁放在一起作为一个占长度更大的大磁铁使用可以让计数更加稳定，且能测量更高的转速(因为高电平的持续时间更长了，不易丢失)。

- sub: 
  - ~/hall_sensor(std_msgs/Bool)
- pub: 
  - ~/wheel_circles(geometry_msgs/TwistStamped): filtered velocity in x, raw velocity in y.

## imu_f

- [x] 修改 odom 的 frame_id
- [x] 提高 odom 的更新频率
- [x] 修改 odom -> base_link 的 tf