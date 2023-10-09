---
title: 装配并学习点云库
description: 本周前三天帮助装配了小无人机，后几天主要在写自主导航的算法
date: 2023-03-26T02:02:45+08:00
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/100285476_p0_master1200.jpg
toc: true
comments: true
hidden: false
draft: false
---

## 周一至周三

由于交付时间有点紧，生产的同事主要在做交付到西藏的大型无人机，然后我就去帮忙装配小无人机

![iDB5Uw](https://i.imgur.com/FASgGfA.jpg)

## 周三到周五

继续我的算法部分，这两天先跑通了`orb-slam3`，`ORB-SLAM`是西班牙 `Zaragoza` 大学的 `Raúl Mur-Arta` 编写的视觉 SLAM 系统。 它是一个完整的 SLAM 系统，包括视觉里程计、跟踪、回环检测，是一种完全基于稀疏特征点的单目 SLAM 系统，同时还有单目、双目、RGBD 相机的接口。核心是使用 ORB (Orinted FAST and BRIEF) 作为整个视觉 SLAM 中的核心特征。

这周首先在我的电脑上跑通了20年发布的`orb-slam3`。

效果如下:

![iDBS3a.jpeg](https://i.imgur.com/D1uUUxm.jpg)

图中左侧窗口为构建的稀疏地图，蓝色表示相机位姿，散点为提取的orb特征点，右侧窗口为双目红外相机实时拍摄到的图像，绿色点为图像中的检测到的特征点

### 运行步骤

主要运行环境为`ubuntu20.04`系统，`ROS`版本为`noetic`，`OpenCV`版本4.5.5，`eigen`版本3.4.0

在运行过程中主要官方文档，其中主要遇到了两个bug，如下

#### opencv版本问题

在运行ROS的实例中，由于安装时`ROS noetic`自带的有4.2.0的`OpenCV`，`cv_bridge`也是4.2的，导致会出现如下错误

```bash
'cv::Exception' what():  OpenCV(4.5.5) /home/ubuntu/Downloads/opencv-4.5.5/opencv/modules/core/src/matrix.cpp:250: error: (-215:Assertion failed) s >= 0 in function 'setSize'`
'cv::Exception' what():  OpenCV(4.5.5) /home/ubuntu/Downloads/opencv-4.5.5/opencv/modules/core/src/alloc.cpp:73: error: (-4:Insufficient memory) Failed to allocate 87219830269440 bytes in function 'OutOfMemoryError
```

需要自己再下载一个`cv_bridge`的源码，然后编译出本机4.5.5版本的`cv_bridge`，随后在`orbslam3`的`CMakeLists.txt`中修改如下

```cmake
set(cv_bridge_DIR "/home/username/path_to_cvbridge_build_ws/devel/share/cv_bridge/cmake")
set(OpenCV_DIR /usr/local/share/opencv4)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.5.5 REQUIRED)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})
```

#### c++版本问题

原版默认的c++为c++11版本，但实操发现编译无法通过，需要修改为c++14，修改如下

```cmake
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
#添加c++14标准
```

#### eigen3

```bash
CMake Error at CMakeLists.txt:44 (find_package):
  Found package configuration file:

    /usr/local/lib/cmake/Pangolin/PangolinConfig.cmake

  but it set Pangolin_FOUND to FALSE so package "Pangolin" is considered to
  be NOT FOUND.  Reason given by package:

  Pangolin could not be found because dependency Eigen3 could not be found.
```

cmakelists.txt中eigen部分换为

```cmake
FIND_PACKAGE(Eigen3 REQUIRED NO_MODULE)
```

#### intel realsense d455相关设置

需要将红外相机的红外发射器关掉，否则会极大影响特征的提取

```xml
stereo_module    emitter_enabled
```

需要将lanuch文件中这两项设为false

## 下一周计划

目前的算法中只能建立起稀疏地图用于自身的定位，下一步需要借助`pcl`库和深度相机建立起稠密点云地图来实现避障和导航功能
