---
title: 使用ublox f9p GPS模块
description: ubuntu下用ROS接受GPS信息
date: 2023-05-05T16:14:45+08:00
categories:
  - GPS
keyword:
  - GPS
  - ros
tags:
  - GPS
  - ros
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/79826536_p0_master1200.jpg
toc: true
math: true
comments: true
hidden: false
draft: false
---



## 编译报错

```bash
/home/tioeare/project/FASTLAB/gps_ws/src/ublox/ublox_gps/src/node.cpp:39:10: fatal error: rtcm_msgs/Message.h: No such file or directory
   39 | #include <rtcm_msgs/Message.h>
      |          ^~~~~~~~~~~~~~~~~~~~~
compilation terminated.
make[2]: *** [ublox/ublox_gps/CMakeFiles/ublox_gps_node.dir/build.make:76: ublox/ublox_gps/CMakeFiles/ublox_gps_node.dir/src/node.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:4154: ublox/ublox_gps/CMakeFiles/ublox_gps_node.dir/all] Error 2
make: *** [Makefile:146: all] Error 2
Invoking "make -j8 -l8" failed
```

下载https://github.com/tilk/rtcm_msgs放到gps_ws/src目录下

```bash
/usr/bin/ld: CMakeFiles/ublox_gps_node.dir/src/node.cpp.o: in function `ublox_node::UbloxNode::configureUblox()':
node.cpp:(.text+0x86b7): undefined reference to `ublox_node::UbloxNode::kResetWait'
/usr/bin/ld: /home/tioeare/project/FASTLAB/gps_ws/devel/lib/libublox_gps.so: undefined reference to `ublox_gps::Gps::kSetBaudrateSleepMs'
collect2: error: ld returned 1 exit status
make[2]: *** [ublox/ublox_gps/CMakeFiles/ublox_gps_node.dir/build.make:151: /home/tioeare/project/FASTLAB/gps_ws/devel/lib/ublox_gps/ublox_gps] Error 1
make[1]: *** [CMakeFiles/Makefile2:4505: ublox/ublox_gps/CMakeFiles/ublox_gps_node.dir/all] Error 2
make: *** [Makefile:146: all] Error 2
Invoking "make -j8 -l8" failed
```

将`src/ublox/ublox_gps/include/ublox_gps/gps.h`和`src/ublox/ublox_gps/src/node.cpp`中`constexpr static int kSetBaudrateSleepMs = 500;`和`constexpr static int kResetWait = 10;`移到Gps或UbloxNode类外
