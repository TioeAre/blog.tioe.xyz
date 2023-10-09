---
title: D455标定与pcl点云库的学习
description:
date: 2023-03-26T02:02:45+05:00
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/102368197_p0_master1200.jpg
toc: true
comments: true
hidden: false
draft: false
---

# RealSense D455的标定

## imu自校准

intel发布的d455深度相机自带的有一个imu，在对相机进行标定之前首先要校准一下imu

校准操作见[官方文档](https://dev.intelrealsense.com/docs/imu-calibration-tool-for-intel-realsense-depth-camera)，官方提供有python脚本用于自动化校准，pdf中对具体操作有明确表述。这里提到一点操作时的一个bug

正常情况下执行校准程序输出应当类似于

![iUc3Py](https://i.imgur.com/IpKhwuV.png)

只要按照pdf中示例的方式摆放相机，当三轴角度都为true的时候就会自动收集数据，但是我最开始遇到了

![iUcLPq](https://i.imgur.com/RPZEi6f.png)

一直停留在`Status.rotate`状态，无论怎么转都无法稳定在`collect`中，然后在github的issue中发现了相似问题

解决方法：

修改`rs-imu-calibration.py`第`100`行与`616`行，范围改大一些到0.6就可以了

```python
self.max_norm = np.linalg.norm(np.array([0.6, 0.6, 0.6]))

	max_norm = np.linalg.norm(np.array([0.6, 0.6, 0.6]))
```

参考链接:https://github.com/IntelRealSense/librealsense/issues/10418#issuecomment-1103125414

打开`realsense-viewer`可以发现校准后竖直摆放相机，重力加速度近似于9.8，而校准前显示为8.8，说明校准起到效果

![calibration.png](https://i.imgur.com/8eujYY2.png)

## 联合标定

出现了以下bug，需要`sudo apt install libelf-dev`

```bash
/home/username/packages/realsense_imu_catkin_ws/src/code_utils/include/code_utils/backward.hpp:216:25: fatal error: elfutils/libdw.h: No such file or directory
  216 | #               include <elfutils/libdw.h>
      |                         ^~~~~~~~~~~~~~~~~~
```

但是安装的时候apt报错

```bash
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies.
 libdw-dev : Depends: libelf-dev but it is not going to be installed
             Depends: libdw1 (= 0.165-3ubuntu1.2) but 0.176-1.1build1 is to be installed
E: Unable to correct problems, you have held broken packages.
```

下载最新的安装包手动安装

 http://archive.ubuntu.com/ubuntu/pool/main/e/elfutils/libelf-dev_0.176-1.1build1_amd64.deb

http://archive.ubuntu.com/ubuntu/pool/main/e/elfutils/libdw-dev_0.176-1.1build1_amd64.deb

```bash
/home/username/packages/realsense_imu_catkin_ws/src/code_utils/src/sumpixel_test.cpp:2:10: fatal error: backward.hpp: No such file or directory
    2 | #include "backward.hpp"
      |          ^~~~~~~~~~~~~~
compilation terminated.
```

修改`sumpixel_test.cpp`第2行为`#include "code_utils/backward.hpp"

```bash
error: ‘integer_sequence’ is not a member of ‘std’ 75 | std::integer_sequence<T, N, Ns...>
```

修改CMakests.txt

```cmake
set(CMAKE_CXX_STANDARD 14)
```

```bash
/home/username/packages/realsense_imu_catkin_ws/src/code_utils/src/mat_io_test.cpp:33:47: error: ‘CV_LOAD_IMAGE_UNCHANGED’ was not declared in this scope
   33 |     Mat img1 = imread( "/home/gao/IMG_1.png", CV_LOAD_IMAGE_UNCHANGED );
      |                                               ^~~~~~~~~~~~~~~~~~~~~~~
```

opencv版本宏定义问题，改相关代码为新版的就行

https://blog.csdn.net/weixin_44675820/article/details/124796674

https://www.cnblogs.com/zzzsj/p/15699524.html

```bash
/home/tioeare/packages/realsense_imu_catkin_ws/src/imu_utils/src/imu_an.cpp:68:19: error: aggregate ‘std::ofstream out_t’ has incomplete type and cannot be defined
```

修改`imu_an.cpp`，添加`#include <fstream>`

```bash
Errors     << aslam_time:make /home/tioeare/packages/kalibr_workspace/logs/aslam_time/build.make.000.log
/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_cv/aslam_time/src/time.cpp:84:25: error: ISO C++17 does not allow dynamic exception specifications
   84 |                         throw (NoHighPerformanceTimersException)
      |                         ^~~~~
make[2]: *** [CMakeFiles/aslam_time.dir/build.make:76: CMakeFiles/aslam_time.dir/src/time.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:295: CMakeFiles/aslam_time.dir/all] Error 2
make: *** [Makefile:146: all] Error 2
```

把代码里这一行注释掉，很奇怪，这种用法应该C++11就禁止了，github上也没人提

```bash
gyr x  numData 240384
gyr x  start_t 1.6799e+09
gyr x  end_t 1.6799e+09
gyr x dt
-------------600.003 s
-------------10 min
-------------0.166667 h
gyr x  freq 400.636
gyr x  period 0.00249603
gyr y  numData 240384
gyr y  start_t 1.6799e+09
gyr y  end_t 1.6799e+09
gyr y dt
-------------600.003 s
-------------10 min
-------------0.166667 h
gyr y  freq 400.636
gyr y  period 0.00249603
gyr z  numData 240384
gyr z  start_t 1.6799e+09
gyr z  end_t 1.6799e+09
gyr z dt
-------------600.003 s
-------------10 min
-------------0.166667 h
gyr z  freq 400.636
gyr z  period 0.00249603
Gyro X
C   -2.40087    59.1038   -17.8144    3.38392 -0.0618206
 Bias Instability 3.00267e-05 rad/s
 Bias Instability 5.92447e-05 rad/s, at 26.0036 s
 White Noise 16.8009 rad/s
 White Noise 0.00479981 rad/s
  bias 0.0580428 degree/s
-------------------
Gyro y
C   -3.7193   89.2566  -24.0407   4.84558 -0.260302
 Bias Instability 2.69712e-06 rad/s
 Bias Instability 9.61604e-05 rad/s, at 22 s
 White Noise 26.42 rad/s
 White Noise 0.00733927 rad/s
  bias -0.304682 degree/s
-------------------
Gyro z
C   -1.0341   30.2021  -13.4864   3.95908 -0.244715
 Bias Instability 4.5525e-06 rad/s
 Bias Instability 1.78885e-05 rad/s, at 56.7372 s
 White Noise 7.71113 rad/s
 White Noise 0.00214285 rad/s
  bias -0.095305 degree/s
-------------------
==============================================
==============================================
acc x  numData 240384
acc x  start_t 1.6799e+09
acc x  end_t 1.6799e+09
acc x dt
-------------600.003 s
-------------10 min
-------------0.166667 h
acc x  freq 400.636
acc x  period 0.00249603
acc y  numData 240384
acc y  start_t 1.6799e+09
acc y  end_t 1.6799e+09
acc y dt
-------------600.003 s
-------------10 min
-------------0.166667 h
acc y  freq 400.636
acc y  period 0.00249603
acc z  numData 240384
acc z  start_t 1.6799e+09
acc z  end_t 1.6799e+09
acc z dt
-------------600.003 s
-------------10 min
-------------0.166667 h
acc z  freq 400.636
acc z  period 0.00249603
acc X
C  -3.7565e-05   0.00153459 -0.000440891  5.14013e-05  1.36731e-05
 Bias Instability 0.000415133 m/s^2
 White Noise 0.0229567 m/s^2
-------------------
acc y
C -0.000153981   0.00273578  0.000838508 -0.000456266  4.36107e-05
 Bias Instability 0.000674808 m/s^2
 White Noise 0.0392674 m/s^2
-------------------
acc z
C -4.04458e-05   0.00242156  -0.00204122  0.000559034 -3.01212e-05
 Bias Instability 0.000795269 m/s^2
 White Noise 0.0348949 m/s^2
-------------------
[imu_an-1] process has finished cleanly
```

```bash
Traceback (most recent call last):
  File "build/kalibr/atomic_configure/kalibr_calibrate_cameras", line 15, in <module>
    exec(compile(fh.read(), python_script, 'exec'), context)
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_cameras", line 465, in <module>
    main()
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_cameras", line 175, in main
    cam = kcc.CameraGeometry(cameraModel, targetConfig, dataset, verbose=(parsed.verbose or parsed.showextraction))
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraCalibrator.py", line 48, in __init__
    self.ctarget = TargetDetector(targetConfig, self.geometry, showCorners=verbose)
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraCalibrator.py", line 81, in __init__
    targetParams = targetConfig.getTargetParams()
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py", line 187, in func
    return f(*args, **kwargs)
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py", line 620, in getTargetParams
    errList.append("invalid tagSpacing (float)")
NameError: name 'errList' is not defined
```

相应位置的文件中errList全部换成途中的print语句

```bash
Traceback (most recent call last):
  File "build/kalibr/atomic_configure/kalibr_calibrate_imu_camera", line 15, in <module>
    exec(compile(fh.read(), python_script, 'exec'), context)
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera", line 247, in <module>
    main()
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera", line 173, in main
    chain.printDetails()
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py", line 765, in printDetails
    camConfig.printDetails(dest)
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py", line 387, in printDetails
    dist_model, dist_coeff = self.getDistortion()
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py", line 187, in func
    return f(*args, **kwargs)
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py", line 355, in getDistortion
    self.checkDistortion(self.data["distortion_model"],
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py", line 350, in checkDistortion
    self.raiseError("distortion model '{}' requires {} coefficients; {} given".format(
  File "/home/tioeare/packages/kalibr_workspace/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py", line 235, in raiseError
    raise RuntimeError( "{0}{1}".format(header, message) )
RuntimeError: [CameraConfig Reader]: distortion model 'radtan' requires 4 coefficients; 1 given
```

chain.yaml加逗号

```c++
pcl::transformPointCloud(*cloud_voxel_tem, *cloud1, T.matrix());
//报错
Signal: SIGSEGV (Segmentation fault)
[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
Assignment with new_width equal to 0,setting width to size of the cloud and height to 1
```

