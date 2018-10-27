# localization_0725

> `describe:` 该模块中的 `interface` 规定了定位模块的接口形式，

## pf_loocalization

> `describe:` 基于粒子滤波实现的定位算法，参考了 `MIT Racecar` 的 [particle filter localization](https://github.com/mit-racecar/particle_filter#particle-filter-localization) 以及 `ros navigation` 中的 `amcl` 包。主要是结合了 `MIT` 中的 `rangelibc` 和 `amcl` 中的 `kdtree` 。

- **service:**
    - `GetMap`: call `map_server` 提供的 `nav_msgs::GetMap` service ，获得 `static_map`
- **subscribe:**
    - `/initialpose`: `rviz` 中给出的小车位置先验
    - `/scan`: `lidar` 提供的 `measurement`
    - `/odom`: 里程计，小车的运动估计，一般可以由 `轮速计`、`IMU`等传感器计算粗略得到。本系统中使用的是 `LOAM` 在 `transformMaintenance` 中发布的 `/integrated_to_init` ，优点是计算结果比较精确，缺点则是计算复杂，占据资源较多
- **publish:**
    - `/pf/odom`: `pf_localization` 计算的小车的位置估计，也可以视为`tf`: `map -> laser`
    - `/pf/particles`: `PoseArray`, 实时的粒子(100 个)信息
    - `/tf`: `map -> laser`


---

### pc2scan

> `describe:` 根据点云的坐标确定其所属的 `ScanID` ，将其保存为 `sensor_msgs::LaserScan` 。
> `usage:` `rosrun localization_0725 pc2scan <input.bag> <laser_topic> <output.bag>`


## 三维点云定位

> 三维点云的蒙特卡洛定位

#### 要点

1. 在转角上，除了 z 轴， x、y 一般不会很大（否则车就会倾覆）
2. 在位置上，相对于 x、y 轴， z 轴的变化一般比较小
3. 如果是基于蒙特卡洛定位的话，需要重新设计 `measurement model` 以及 `motion model` ，在系统添加了 `IMU` 之后， motion model 可以快速地进行更新，
4. 注意 measurement model 最好要进行滤波，因为点云数目较大，每个点进行匹配可能耗时太长而且匹配效果不好；或者是提取点云中的特征进行匹配
5. 在点云地图的生成上，我们可以结合多个地图将动态物体和静态物体进行区分，这样在 measurement model 中，可以为静态物体的匹配分配更高的权重，在实际实现中，我们可能需要为点云地图中的每个点赋予一个权重
6. 蒙特卡洛定位中的 `resample` 步骤并不一定是每次都需要，可以在某些指标的指导下进行
7. 对于路径规划所需要的地图，目前不需要全部的点云地图，可以从点云地图中分割出地面这些可行区域（`pcl` 中提供了点云分割的工具，可以了解一下），然后定位的初始化则可以基于该地图给出(x, y, yaw)，z 可以初始化为车辆的中心高度

#### Info

1. autoware 的点云定位是基于 `ICP` 方法的，可以学习