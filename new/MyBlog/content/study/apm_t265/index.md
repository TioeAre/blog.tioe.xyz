---
title: APM中t265数据解析
description:
date: 2023-05-15T16:14:45+08:00
categories:
  - Multi-Sensor
keyword:
  - t265
tags:
  - t265
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/74923161_p0_master1200.jpg
toc: true
math: true
comments: true
hidden: false
draft: false

---



## libraries/AP_VisualOdom/AP_VisualOdom.h

参数有相机的类别, 相机相对飞控的位移, 相机相对飞控摆放的方向(前后左右向下), 相机数据的缩放系数, 延迟, 速度\位置\yaw的噪声

```c++
float posErr = 0;
float angErr = 0;
if (!isnan(m.pose_covariance[0])) {
    posErr = cbrtf(sq(m.pose_covariance[0])+sq(m.pose_covariance[6])+sq(m.pose_covariance[11]));
    angErr = cbrtf(sq(m.pose_covariance[15])+sq(m.pose_covariance[18])+sq(m.pose_covariance[20]));
}
// 噪声直接是将协方差矩阵中的相加
```



主要函数有

1.   `init()`

     ```c++
         // create backend
         switch (VisualOdom_Type(_type.get())) {
         case VisualOdom_Type::None:
             // do nothing
             break;
     #if AP_VISUALODOM_MAV_ENABLED
         case VisualOdom_Type::MAV:
             _driver = new AP_VisualOdom_MAV(*this);
             break;
     #endif
     #if AP_VISUALODOM_INTELT265_ENABLED
         case VisualOdom_Type::IntelT265:
         case VisualOdom_Type::VOXL:
             _driver = new AP_VisualOdom_IntelT265(*this);
             break;
     #endif
         }
     ```

     如果是t265的话默认成VOXL, 这两个应该用的是类似的数据

2.   `handle_vision_position_delta_msg()`

     调用_driver->handle_vision_position_delta_msg()获取角度和位置数据

3.   `handle_vision_position_estimate()`

     获取四元数或者三轴角度然后调用_driver->handle_vision_position_estimate()进行位置估计

## libraries/AP_VisualOdom/AP_VisualOdom_IntelT265.cpp

1.   `handle_vision_position_estimate()`

     ```c++
     handle_voxl_camera_reset_jump(pos, att, reset_counter);
     // 先调用判断传来的数据有没有跳变, 如果有跳变的话就重设参数, 该参数是从mavlink接受来的, 并不是飞控自己判断的
     // 然后直接将获取到的位姿发送给导航或传感器等
     rotate_and_correct_position(pos);
     rotate_attitude(att);
     // 矫正位姿
     ```

2.   `handle_vision_speed_estimate()`

     ```c++
     // rotate velocity to align with vehicle
     Vector3f vel_corrected = vel;
     rotate_velocity(vel_corrected)
     ```

关于上位机传来的数据并没有怎么处理, 仅仅将位移和速度矫正了一下

