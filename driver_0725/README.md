# 

## hall_sensor

> desc: 根据霍尔传感器计算转速，但目前不知道是程序问题还是什么原因，圈数的计数存在一定的波动，所以在程序中增加了一个 kalman filter 并对速度进行了平滑处理。


- sub: 
  - ~/hall_sensor(std_msgs/Bool)
- pub: 
  - ~/wheel_circles(geometry_msgs/TwistStamped): filtered velocity in x, raw velocity in y.