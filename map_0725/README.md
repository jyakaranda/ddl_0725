#

## map_filter

> 主要是对点云进行滤波，包括：对建图过程中的动态物体进行滤波、对过于稀疏的点云进行滤波等

动态物体的滤波方法主要有 `ICP`、`NDT` 以及最近在 ICRA 上发表的一篇基于混合模型的点云配准论文等，而点云的配准则可以以 `IMU` 的积分结果作为旋转平移矩阵的初始输入

稀疏点云的滤波目前实现的比较简单，因为还没有确定这种滤波是否必要

这些滤波主要是在建图前对原始点云进行处理，然后再作为 LOAM 等 SLAM 算法的输入

在建图之后是否还需要滤波目前还没有确定

#### 动态物体的滤波

- 对两帧点云计算 `transform` ，将点云中移动距离与 `transform` 相差较大的点进行剔除
- 在点云中提取 `bounding box` ，基于 `bounding box` 进行配准
- 看下相关论文


pointcloud_to_pcd

Subscribes to a ROS topic and saves point cloud messages to PCD files. Each message is saved to a separate file, names are composed of an optional prefix parameter, the ROS time of the message, and the .pcd extension.

Subscribed Topics
<br>
input (sensor_msgs/PointCloud2) 
</br>A stream of point clouds to save as PCD files.

Parameters
</br>
~prefix (str)
</br>
Prefix for PCD file names created.
</br>
~fixed_frame (str)
</br>
If set, the transform from the fixed frame to the frame of the point cloud is written to the VIEWPOINT entry of the pcd file.
</br>
~binary (bool, default: false)
</br>
Output the pcd file in binary form.
</br>
~compressed (bool, default: false)
In case that binary output format is set, use binary compressed output.

pcd_to_pointcloud

Loads a PCD file, publishing it one or more times as a ROS point cloud message.

Published Topics
</br>
cloud_pcd (sensor_msgs/PointCloud2)
</br>
A stream of point clouds generated from the PCD file.

Parameters
</br>
~frame_id (str, default: /base_link)
</br>
Transform frame ID for published data.

### rt_filter_node

对点云进行过滤，只保留一定范围内的点云数据