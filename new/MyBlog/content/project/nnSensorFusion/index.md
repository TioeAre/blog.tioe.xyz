---
title: 使用神经网络融合位置数据(第一版)
description:
date: 2023-06-17T19:30:45+08:00
categories:
  -
keywords:
  -
tags:
  -
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/84719437_p0.jpg
toc: true
comments: true
hidden: false
draft: false

---

本文受NICE-SLAM启发, 最初是想要设计一个网络, 使其能够将不同传感器的数据融合起来得到一个准确的无人机位置. 首先设计并训练一个用来融合数据的网络, 训练出的网络作为拟合位置的函数, 然后用神经隐式表达的方法, 给出一个无人机位置的先验, 然后用刚才预训练好的网络来decode出不同传感器可能的读数, 再将该读数与真实读到的数据做loss, 反向梯度计算出在什么样的位置下最有可能用这几种传感器读到这些读数. 从而得到最大似然的无人机位置.

然后首先便需要训练一个能够起到decode作用的网络. 在训练之前我想要先训练一个正向融合数据的网络作为测试, 然后想到了FCN全卷积网络, 当时觉得全卷积网络不限置输入和输出的大小, 训练的是卷积核, 似乎刚好符合我想要训练一个融合不同传感器数值的函数的目的.

为了训练网络, 首先需要数据. 在网上找到了t265的xacro的模型, 根据该模型将其转换为gazebo的px4无人机仿真中使用的sdf模型, 具体文件如下

```xml
<?xml version="1.0"?>
<sdf version='1.5'>

  <model name='realsense_t265'>
    <link name='base_link'>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.055</mass>
        <inertia>
          <ixx>9.108e-05</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>2.51e-06</iyy>
          <iyz>0</iyz>
          <izz>8.931e-05</izz>
        </inertia>
      </inertial>
      <collision name='base_link_fixed_joint_lump__t265_pose_frame_collision'>
        <pose>0 0 0 0 -0 0</pose>
        <geometry>
          <box>
            <size>0.013 0.108 0.024</size>
          </box>
        </geometry>
      </collision>
      <visual name='base_link_fixed_joint_lump__t265_pose_frame_visual'>
        <pose>0 0 0 1.57 -0 1.57</pose>
        <geometry>
          <mesh>
            <scale>0.001 0.001 0.001</scale>
            <uri>model://realsense_t265/meshes/realsense_t265.stl</uri>
          </mesh>
        </geometry>
      </visual>
      <sensor name='camera1' type='wideanglecamera'>
        <camera>
          <horizontal_fov>6.283</horizontal_fov>
          <image>
            <width>800</width>
            <height>848</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <lens>
            <type>custom</type>
            <custom_function>
              <c1>1.05</c1>
              <c2>4</c2>
              <f>1</f>
              <fun>tan</fun>
            </custom_function>
            <scale_to_hfov>1</scale_to_hfov>
            <cutoff_angle>3.1415</cutoff_angle>
            <env_texture_size>512</env_texture_size>
          </lens>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
        </camera>
        <plugin name='camera_controller1' filename='libgazebo_ros_camera.so'>
          <alwaysOn>1</alwaysOn>
          <updateRate>30</updateRate>
          <cameraName>t265/fisheye1</cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>"t265_fisheye1_optical_frame"</frameName>
          <hackBaseline>0.07</hackBaseline>
          <distortionK1>-0.007419134024530649</distortionK1>
          <distortionK2>0.041209351271390915</distortionK2>
          <distortionK3>-0.03811917081475258</distortionK3>
          <distortionT1>0.006366158835589886</distortionT1>
          <distortionT2>0.0</distortionT2>
          <CxPrime>416.00531005859375</CxPrime>
          <Cx>16.00531005859375</Cx>
          <Cy>403.38909912109375</Cy>
        </plugin>
        <pose>0.01 0.042 0 0 -0 0</pose>
      </sensor>

      <sensor name='camera2' type='wideanglecamera'>
        <camera>
          <horizontal_fov>6.283</horizontal_fov>
          <image>
            <width>800</width>
            <height>848</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <lens>
            <type>custom</type>
            <custom_function>
              <c1>1.05</c1>
              <c2>4</c2>
              <f>1</f>
              <fun>tan</fun>
            </custom_function>
            <scale_to_hfov>1</scale_to_hfov>
            <cutoff_angle>3.1415</cutoff_angle>
            <env_texture_size>512</env_texture_size>
          </lens>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
        </camera>
        <plugin name='camera_controller2' filename='libgazebo_ros_camera.so'>
          <alwaysOn>1</alwaysOn>
          <updateRate>30</updateRate>
          <cameraName>t265/fisheye2</cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>t265_fisheye2_optical_frame</frameName>
          <hackBaseline>0.07</hackBaseline>
          <distortionK1>-0.007419134024530649</distortionK1>
          <distortionK2>0.041209351271390915</distortionK2>
          <distortionK3>-0.03811917081475258</distortionK3>
          <distortionT1>0.006366158835589886</distortionT1>
          <distortionT2>0.0</distortionT2>
          <CxPrime>416.00531005859375</CxPrime>
          <Cx>16.00531005859375</Cx>
          <Cy>403.38909912109375</Cy>
        </plugin>
        <pose>0.01 -0.022 0 0 -0 0</pose>
      </sensor>

      <gravity>1</gravity>
      <sensor name='t265_imu' type='imu'>
        <always_on>1</always_on>
        <update_rate>500</update_rate>
        <visualize>0</visualize>
        <plugin name='imu_plugin' filename='libgazebo_ros_imu_sensor.so'>
          <topicName>t265/gyro/sample</topicName>
          <bodyName>t265_pose_frame</bodyName>
          <updateRateHZ>500.0</updateRateHZ>
          <gaussianNoise>0.000001</gaussianNoise>
          <xyzOffset>0 0 0</xyzOffset>
          <rpyOffset>0 0 0</rpyOffset>
          <frameName>t265_link</frameName>
        </plugin>
        <pose>0 0 0 0 -0 0</pose>
      </sensor>

      <plugin name='p3d_base_controller' filename='libgazebo_ros_p3d.so'>
        <alwaysOn>1</alwaysOn>
        <updateRate>500</updateRate>
        <topicName>t265/odom/sample</topicName>
        <gaussianNoise>0.001</gaussianNoise>
        <frameName>world</frameName>
        <xyzOffsets>0 0 0</xyzOffsets>
        <rpyOffsets>0 0 0</rpyOffsets>
        <bodyName>base_link</bodyName>
        <xyzOffset>0 0 0</xyzOffset>
        <rpyOffset>0 -0 0</rpyOffset>
        <ignition::corrected_offsets>1</ignition::corrected_offsets>
      </plugin>
      <frame name='t265_accel_joint' attached_to='t265_link'>
        <pose>0 0 0 0 -0 0</pose>
      </frame>
      <frame name='t265_accel_frame' attached_to='t265_accel_joint'/>
      <frame name='t265_fisheye1_optical_joint' attached_to='t265_fisheye1_frame'>
        <pose>0.01 0 0 0 -0 0</pose>
      </frame>

      <frame name='t265_fisheye1_optical_frame' attached_to='t265_fisheye1_optical_joint'/>
      <frame name='t265_fisheye1_joint' attached_to='t265_link'>
        <pose>0 0.042 0 0 -0 0</pose>
      </frame>
      <frame name='t265_fisheye1_frame' attached_to='t265_fisheye1_joint'/>
      <frame name='t265_fisheye2_optical_joint' attached_to='t265_fisheye2_frame'>
        <pose>0.01 0 0 0 -0 0</pose>
      </frame>
      <frame name='t265_fisheye2_optical_frame' attached_to='t265_fisheye2_optical_joint'/>
      <frame name='t265_fisheye2_joint' attached_to='t265_link'>
        <pose>0 -0.022 0 0 -0 0</pose>
      </frame>
      <frame name='t265_fisheye2_frame' attached_to='t265_fisheye2_joint'/>

      <frame name='t265_gyro_joint' attached_to='t265_link'>
        <pose>0 0 0 0 -0 0</pose>
      </frame>
      <frame name='t265_gyro_frame' attached_to='t265_gyro_joint'/>
      <frame name='t265_joint' attached_to='t265_pose_frame'>
        <pose>0 0 0 0 -0 0</pose>
      </frame>
      <frame name='t265_link' attached_to='t265_joint'/>
      <frame name='t265_pose_frame_joint' attached_to='t265_odom_frame'>
        <pose>0 0 0 0 -0 0</pose>
      </frame>
      <frame name='t265_pose_frame' attached_to='t265_pose_frame_joint'/>
      <frame name='t265_odom_frame_joint' attached_to='base_link'>
        <pose>0 0 0 0 -0 0</pose>
      </frame>
      <frame name='t265_odom_frame' attached_to='t265_odom_frame_joint'/>

    </link>
  </model>

</sdf>

<!-- vim: set et ft=xml fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : -->

```

在gazebo中构建一个包含由imu, t265, px4flow, gps这几中传感器的四旋翼无人机模型, 然后使用QGroundControl地面站连接飞机进入任务模式, 按照预定的模拟线路在仿真中飞行1小时左右, 期间写了一个python脚本用于ros采集数据并录制了一个rosbag. 为了提高网络的鲁棒性, 在收集数据的过程中随机5%的数据进行清零或增加0-50内的随机数作为该传感器失灵的情况

最终收集到的数据以n\*100\*4\*15的形式储存在json文件中, n指n个batch, 100指这1秒100hz的连续数据, 4指上述四个传感器, 15分别为三维xyz位置, 三维传感器相对于飞机中心点的偏移量, 三维数据是否可用, 三维数据是否发生跳变和三维传感器误差种类. 其中前六维为float32, 后九维为bool

 经过多次修改测试后的网络架构如下图所示

![VNQMUb](https://i.imgur.com/c5mRb2o.png)

代码实现见https://github.com/TioeAre/data_fusion_network

然而最终效果却不尽人意, 网络并没有学习到如何判断某个数据是否正常等条件, 反而最后训练出的网络起到了类似剔除不良数据的作用. 网络输出能够与真实数据保持一致性, 能够剔除掉之前手动加入的大误差或清零部分数据

后续与曹老师的交流发现我的网络结构设计的还是太简单了, 只用位置, 也没有考虑到每个传感器的性质, 没有考虑到姿态.

后续准备再继续阅读学习网络拟合imu的相关论文, 做好设计后再进行尝试
