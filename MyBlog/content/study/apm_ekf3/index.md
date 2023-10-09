---
title: APM中ekf3算法
description:
date: 2023-05-23T16:14:45+08:00
categories:
  - Multi-Sensor
keyword:
  - ekf
tags:
  - ekf
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/79509418_p0_master1200.jpg
toc: true
math: true
comments: true
hidden: false
draft: false

---



![VNQVrt](https://i.imgur.com/K4eS6rE.jpg)

ekf3用于估计位姿, 导航坐标系为NED(北东地)

数学推导参考[PX4和APM导航解算过程推导-24维状态EKF](https://blog.csdn.net/u014527548/article/details/107437177)

matlab见[InertialNav-Github](https://github.com/priseborough/InertialNav)

ekf2代码介绍[EKF2](https://blog.csdn.net/lixiaoweimashixiao/article/details/105900170)

![VNQfhJ](https://i.imgur.com/BZhyhPM.png)

![VNQ4Zc](https://i.imgur.com/NbkcIkh.png)

![VNQWBd](https://i.imgur.com/JcdfQMd.png)

![VNQc3y](https://i.imgur.com/LTAZlbe.png)

![VNQm55](https://i.imgur.com/xPMVByA.png)

![VNQBi8](https://i.imgur.com/Dad1StY.png)

## AP_NavEKF3.cpp

### bool InitialiseFilter()

用于初始化滤波器, 滤波器可以有好几个core, 每个core或lane对应一种传感器和imu的状态估计, 初始化时EK3_PRIMARY可以在为某个core或lane. 但之后可能会在运行时被修改成其他的lane. Affinity允许不设置首要lane, 并用lane error存储不同传感器的差值, 使系统能够效果最好的作为状态估计. lane error随时间积累并通过EK3_ERR_THRESH参数设置non-primary and primary lane比较的阈值, 当超过阈值时会切换用于估计的传感器(下文中的所有core与lane几乎是可以表示同一个东西)

函数中首先获取循环一次的时间, 每个imu对应一个core, 一个imu不能用于多个滤波器, 然后为ekf core申请内存, 如果申请成功的话就调用`NavEKF3_core()`创建每个core, 然后调用`resetCoreErrors()`初始化上文提到的每个core对应的lane error

NavEKF3_core()初始化时使用了`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`中几个函数`InitialiseFilterBootstrap() InitialiseVariables()&&CovarianceInit() InitialiseVariablesMag()`(依次调用), 使系统运行中能够在必要时初始化滤波器(在UpdateFilter()中, 如果filterStatus.value==0的话会调用InitialiseFilterBootstrap()重新初始化)

协方差初始化时, 初始化为对角矩阵

```c++
memset(&P[0][0], 0, sizeof(P));
// define the initial angle uncertainty as variances for a rotation vector
Vector3F rot_vec_var;
rot_vec_var.x = rot_vec_var.y = rot_vec_var.z = sq(0.1f);
// reset the quaternion state covariances
CovariancePrediction(&rot_vec_var);
// velocities
P[4][4]   = sq(frontend->_gpsHorizVelNoise);
P[5][5]   = P[4][4];
P[6][6]   = sq(frontend->_gpsVertVelNoise);
// positions
P[7][7]   = sq(frontend->_gpsHorizPosNoise);
P[8][8]   = P[7][7];
P[9][9]   = sq(frontend->_baroAltNoise);
// gyro delta angle biases
P[10][10] = sq(radians(InitialGyroBiasUncertainty() * dtEkfAvg));
P[11][11] = P[10][10];
P[12][12] = P[10][10];
// delta velocity biases
P[13][13] = sq(ACCEL_BIAS_LIM_SCALER * frontend->_accBiasLim * dtEkfAvg);
P[14][14] = P[13][13];
P[15][15] = P[13][13];
// earth magnetic field
P[16][16] = sq(frontend->_magNoise);
P[17][17] = P[16][16];
P[18][18] = P[16][16];
// body magnetic field
P[19][19] = sq(frontend->_magNoise);
P[20][20] = P[19][19];
P[21][21] = P[19][19];
// wind velocities
P[22][22] = 0.0f;
P[23][23]  = P[22][22];
```

### bool coreBetterScore()

这个函数用于判断是否切换主要的滤波器

```c++
bool NavEKF3::coreBetterScore(uint8_t new_core, uint8_t current_core) const
{
    const NavEKF3_core &oldCore = core[current_core];
    const NavEKF3_core &newCore = core[new_core];
    if (!newCore.healthy()) {
        // never consider a new core that isn't healthy
        return false;
    }
    if (newCore.have_aligned_tilt() != oldCore.have_aligned_tilt()) {
        // tilt alignment is most critical, if one is tilt aligned and
        // the other isn't then use the tilt aligned lane
        return newCore.have_aligned_tilt();
    }
    if (newCore.have_aligned_yaw() != oldCore.have_aligned_yaw()) {
        // yaw alignment is next most critical, if one is yaw aligned
        // and the other isn't then use the yaw aligned lane
        return newCore.have_aligned_yaw();
    }
    // if both cores are aligned then look at relative error scores
    return coreRelativeErrors[new_core] < coreRelativeErrors[current_core];
}
```

先判断新的core是否正常工作, 然后判断这两个core是否有倾斜对齐, 返回有对齐的那一个, 然后判断是否对齐的yaw, 最后再判断两者lane error的大小

### void UpdateFilter()

```c++
struct state_elements {
    QuaternionF quat;           // quaternion defining rotation from local NED earth frame to body frame 0..3
    Vector3F    velocity;       // velocity of IMU in local NED earth frame (m/sec) 4..6
    Vector3F    position;       // position of IMU in local NED earth frame (m)     7..9
    Vector3F    gyro_bias;      // body frame delta angle IMU bias vector (rad)     10..12
    Vector3F    accel_bias;     // body frame delta velocity IMU bias vector (m/sec) 13..15
    Vector3F    earth_magfield; // earth frame magnetic field vector (Gauss)         16..18
    Vector3F    body_magfield;  // body frame magnetic field vector (Gauss)          19..21
    Vector2F    wind_vel;       // horizontal North East wind velocity vector in local NED earth frame (m/sec) 22..23
};
```

先判断两次更新之间是否丢帧, 没有丢帧的话就对每个core UpdateFilter()

然后判断当前的主要core是否正常工作, 如果不正常工作的话就切换core, 此判断只有在主要core正常运行10s后才开始使用

当无人机能够解锁并且主core已经正常运行了超过10s之后进入判断是否要切换primary core(但runCoreSelection这个变量好像只在上面这一个地方被赋值, 也就是说只要此时的primary core正常工作10s之后, 每次UpdateFilter()运行时都会判断是否要切换core)

然后开始计算lane error,

lane error计算的是每个传感器的观测值velPosObs与ekf滤波后的状态变量stateStruct中的值相减之后做平方和(即innovation)再除以方差的和, 感觉本质上就是把这个传感器的数据跟对应的imu的数据做个比较, 如果二者相差大的话就说明这个core没有正常工作

计算每个lane error之后再计算了其他的lane error与primary lane error之间的差值, 并将差值累加起来

如果这个差值小于阈值的话并且距离上一次切换时间超过10s的话并且yaw已经对齐了的话, 就发生切换并重置累加起来的每个lane error

更新滤波器的步骤主要也是调用了`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`中的`UpdateFilter()`函数

NavEKF3_core::UpdateFilter()中先调用update_sensor_selection()更新每个core中传感器的选择, 具体步骤以update_gps_selection()为例

```c++
const auto &gps = dal.gps();	// 先获取gps的数据对象, 储存当前的gps信息
selected_gps = gps.primary_sensor();	// 设置当前的gps索引
preferred_gps = selected_gps;
if (frontend->_affinity & EKF_AFFINITY_GPS) {	// 按位与判断gps是否符合affinity
    if (core_index < gps.num_sensors() ) {
        preferred_gps = core_index;
    }	// 判断是否有超过core_index个gps, 也就是当前的core是否有对应的gps
    if (gps.status(preferred_gps) >= AP_DAL_GPS::GPS_OK_FIX_3D) {	// 检查当前的第code_index个gps也就是当前core对应的第n个gps(如果有的话)的数据是否满足gps的3D定位的标准, 如果满足的话就改用当前core_index的gps
        selected_gps = preferred_gps;
    }
}
```

然后调用controlFilterModes()检查飞行状态(电机状态, 飞行模式, 是否自动设置风力和磁场变化, 是否对齐飞行姿态, 设置导航辅助模式(导航辅助模式主要就是判断用了哪些传感器, 如果用了gps或气压计或视觉定位的话就是绝对测量, 否则用了光流或者机体坐标系原始数据的话设置为相对测量))

然后调用readIMUData()读取imu的数据

```c++
dtIMUavg = 0.02f * constrain_ftype(ins.get_loop_delta_t(),0.5f * dtIMUavg, 2.0f * dtIMUavg) + 0.98f * dtIMUavg;	// 先计算imu频率时使用一个低通滤波
```

然后判断切换加速度计和陀螺仪, 如果切换的话直接赋值原先的偏差到切换来的上. 然后判断运动状态, 主要代码计算了陀螺仪和加速度计的的差值和差值的比例, 然后判断当前是否在地面上并且没有运动

learnInactiveBiases()中主要是对陀螺仪和加速度计的偏差进行修正和赋值, 如果是activate gyro或accel的话就直接将ekf估计出的bias复制给他, 如果是inactivate的话就计算出他与activate之间的误差然后乘一个很小的缩放系数慢慢的将他对齐到activate的

之后读取累积的imu数据, 然后矫正传感器误差

开始更新时首先调用UpdateStrapdownEquationsNED()对imu的数据进行累加和转换, 转换到导航坐标系, 然后计算了imu测量出的各轴速度和加速度

之后调用CovariancePrediction()更新协方差的先验

调用runYawEstimatorPrediction()更新加速度计和陀螺仪对yaw的估计, 对齐姿态角

调用SelectMagFusion()选择融合用的磁力计数据, 开始也是通过yaw source来作为姿态角的一个参照, 调用fuseEulerYaw()融合yaw并计算bias, 然后读取磁力计数据

**SelectVelPosFusion()**开始融合速度和位置数据, 调用CorrectGPSForAntennaOffset()上一时刻的速度减去用delta角度除以时间得到当前时刻的gps速度, 同理得到z轴高度

```c++
    // 判断gps是否可用, pose source是否为gps
    if (gpsDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (posxy_source == AP_NavEKF_Source::SourceXY::GPS)) {
	// 判断是否融合gps的pose或vel
        fuseVelData = frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::GPS);
        fusePosData = true;
#if EK3_FEATURE_EXTERNAL_NAV
        extNavUsedForPos = false;
#endif
	// 把gps的有延迟的观测数据复制出来
        if (fuseVelData) {
            velPosObs[0] = gpsDataDelayed.vel.x;
            velPosObs[1] = gpsDataDelayed.vel.y;
            velPosObs[2] = gpsDataDelayed.vel.z;
        }
        const Location gpsloc{gpsDataDelayed.lat, gpsDataDelayed.lng, 0, Location::AltFrame::ABSOLUTE};
        const Vector2F posxy = EKF_origin.get_distance_NE_ftype(gpsloc);
        velPosObs[3] = posxy.x;
        velPosObs[4] = posxy.y;
#if EK3_FEATURE_EXTERNAL_NAV
    } else if (extNavDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (posxy_source == AP_NavEKF_Source::SourceXY::EXTNAV)) {
        // 如果启用了视觉的话观测数据就换成视觉传来的数据
        extNavUsedForPos = true;
        fusePosData = true;
        velPosObs[3] = extNavDataDelayed.pos.x;
        velPosObs[4] = extNavDataDelayed.pos.y;
#endif // EK3_FEATURE_EXTERNAL_NAV
    }
```

得到数据之后先调用selectHeightForFusion()选取要融合的高度source, 这个函数前半部分也都是在判断读取气压计, gps或其他的数据, 计算传感器测量数据与ekf估计之间的偏移. 在gps数据可以融合的时候用贝叶斯滤波融合gps的数据到ekf的数据和方差里

然后将数据放到ekf的观测里, 然后判断如果用gps并且可用并且需要reset的话, 把ekf的状态量的位置改为gps测得的数据

然后又是一堆判断之后调用了最重要的**FuseVelPosNED()**来融合数据

先判断赋值观测误差, 然后计算ekf的状态量与观测值之间的差值用于判断是否丢帧

如果可以融合数据的话就先计算新息



待更新
