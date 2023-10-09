---
title: 调试室内定位
description: 实验
date: 2023-06-15T19:30:45+08:00
categories:
  - 实验记录
keywords:
  - 实验记录
tags:
  - 实验记录
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/103449092_p0_master1200.jpg
toc: true
comments: true
hidden: false
draft: false

---

**一、实验目的**

1.   使用光流，无人机定位模式
2.   找无人机突然失控的原因

**二、实验过程记录**

1.   首先使用姿态模式起飞，无异常情况
2.   换为位置模式起飞，起飞后存在左右小幅度摇摆偏移
3.   降落未断电后重新起飞，无人机失控向左后方偏移然后遥控器急停
4.   未断电重新尝试解锁，不能解锁并报错如下图

![VNYnj8](https://i.imgur.com/zGlecoi.png)

5.   断电后可以解锁，位置模式仍然失控

6.   重新使用姿态起飞，姿态仍然正常

**三、实验结果**

1.   查看日志发现姿态估计结果异常，飞机水平放置时，飞控仍会以为飞机存在水平倾斜，判断是姿态估计出错导致的失控
2.   还发现报错imu stopped aidding，看源码发现在ekf滤波开始前有相关判断，并在下图两处为PV_AidingMode赋值为AID_NONE然后报错，推测报错原因为参数里ek3_src1_yaw没有赋值

![VNY3lo](https://i.imgur.com/EJfYKJc.png)

![VNYcMV](https://i.imgur.com/fqmfaM1.png)

![img3](https://i.imgur.com/Aihh024.png)

![VNYXoq](https://i.imgur.com/QkNONC5.png)

3.   继续跳转发现AID_NONE会导致滤波结果重置和yaw方向的融合，推测是这个原因导致的姿态估计错误

![VNYXoq](https://i.imgur.com/Qd9MI3T.png)

![VNY78w](https://i.imgur.com/UMiwia8.png)

4.   然而ek3_src1_yaw这个参数只有none, Compass, GPS, GPS with Compass Fallback, ExternalNav, GSF几种，只能选compass，但是又会报错compass unhealthy，尚未解决

2023.06.24:

后续发现kakute mini飞控中并没有磁力计, 板载imu只有加速度记和陀螺仪. 外置磁力计后能够跳出该判断条件, 避免估计的状态量和观测值清零. 在大角度旋转无人机, 状态因光流测算异常而估计错误后能够自动返回正确的姿态

此时仍存在室内定位漂移以及使用光流的激光测高模块会使高度估计失控的情况, 下一步买回来自带陀螺仪补偿的px4flow后再进行实验找到问题
