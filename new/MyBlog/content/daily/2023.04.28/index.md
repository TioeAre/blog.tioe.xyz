---
title: imu补偿光流
description: 通过无人机板载imu补偿光流在pitch和roll上的偏差
date: 2023-04-28T02:02:45+08:00
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/85626613_p0_master1200.jpg
toc: true
comments: true
hidden: false
draft: false
---

# Ardupilot补偿光流

## 问题概述

主要问题: 光流传感器假定无人机为完全水平飞行, 当无人机姿态发生微小倾斜时, 光流认为此时无人机有位移, 进而输出该位移数据, 导致Loiter模式下出现左右浮动

衍生问题: 1.光流模块并不在无人机旋转中心, 当无人机位姿发生变化时, 光流模块的旋转角速度与imu的角速度不一定相同. 2.光流数据的频率与imu发布频率不一定相同, 中间可能出现时间差影响补偿效果

## 融合方案

### 代码修改

1.   获取光流得到的x轴和y轴速度
2.   获取加速度计得到的pitch和roll轴速度
3.   将加速度计得到的数据转化为光流坐标系下的速度
4.   光流速度减去加速度计速度即可

参考[4], [5]中的代码

```c++
///参考资料[4]
Flow_Decode(&Flow_Buffer[0]);	// 光流数据解析
TimePeriod(&flow_Delta);	// 得到这段代码运行时间ms
Flow_Delta_T = Flow_Delta.Time_Delta/1000.0f;	// 得到周期时间

// 得到陀螺仪数据角速度(弧度)
gyro_y = imu.gyroRaw[1] * 180.0f / M_PI_F;	// gyro[1]  pitch光流Y轴
gyro_x = imu.gyroRaw[0] * 180.0f / M_PI_F;	// gyro[0]  roll光流X轴

// 陀螺仪滤波
LPF_1_(8.0f, Flow_Delta_T, gyro_y, gyro_lpf_y);	// gyro low pass filter (delay) for fitting flow data()
LPF_1_(8.0f, Flow_Delta_T, gyro_x, gyro_lpf_x);	// 8.0需要调参, 保证光流数值和陀螺仪数值相位相差不多

// 对光流原始数据进行滤波
LPF_1_(30.0f, Flow_Delta_T, flow_dat.x, flow_dat.x);	// 对光流数据进行简单滤波
LPF_1_(30.0f, Flow_Delta_T, flow_dat.y, flow_dat.y);

// 速度计算方法: (H * flow_dat.x / T) * 100   H为高度, 单位m. T为间隔时间, 单位s, 100是m->cm的转换
UPflow_speed_x =  flow_dat.x / Flow_Delta_T ;  // 速度 rad/s
UPflow_speed_y =  flow_dat.y / Flow_Delta_T ;  // 速度 rad/s

/* --------------------利用陀螺仪对光流进行补偿，保证在原地旋转时，光流输出几乎为 0 -----------------*/
// 1.105和1.101系数需要调参, 用来抵消低通滤波带来的幅值减小
UPflow_speed_x = US100_Distance *  (UPflow_speed_x  -  1.105 * LIMIT(((gyro_lpf_x) / 57.295779f), -flow_t2, flow_t2)) * 100.0f;	// 速度 cm/s
UPflow_speed_y = US100_Distance * (UPflow_speed_y  +  1.101 * LIMIT(((gyro_lpf_y) / 57.295779f), -flow_t1, flow_t1)) * 100.0f;
```

其中`Flow_Decode()`, `LPF_1_()`, `LIMIT()`函数由该博主使用的光流模块厂家提供, 用于获取光流速度和滤波等

```c++
/// 参考资料[5]
if (get_flow_quality() != 0) {
  flow_angle_fix_x = +(120.0f * tan(pitch_delay[3]));
  flow_angle_fix_y = -(120.0f * tan(roll_delay[3]));
} else {
  flow_angle_fix_x = 0;
  flow_angle_fix_y = 0;
}
flow_cal_distance_x = flow_ori_distance_x - flow_angle_fix_x; 	// cm
flow_cal_distance_y = flow_ori_distance_y - flow_angle_fix_y; 	// cm
float flow_speed_x = (flow_cal_distance_x - last_distance_x) / 0.02f;
float flow_speed_y = (flow_cal_distance_y - last_distance_y) / 0.02f;
last_distance_x = flow_cal_distance_x;
last_distance_y = flow_cal_distance_y;
```

以上两篇中主要思想都是通过光流的模块的相关函数获取到光流的数据, 然后将该数据乘以一个需要自己调整的参数, 然后再减去从imu获得的数据.

不同点是[4]中直接获取的光流速度然后利用他的模块手册提供的滤波函数对数据进行滤波之后再减去imu补偿量, 得到补偿后的速度. [5]中是先获取速度然后积分成位移, 再用位移减去补偿量然后再微分成速度.(这么做主要考虑到速度不好标定, 但位移相对来说更容易得到测量值用于矫正参数的赋值)



目前使用的MTF-01光流传感器只提供了读取光流速度的相关函数, 如下

```c++
// mtf01.h
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MICOLINK_MSG_HEAD 0xEF
#define MICOLINK_MAX_PAYLOAD_LEN 64
#define MICOLINK_MAX_LEN MICOLINK_MAX_PAYLOAD_LEN + 7

/*
    消息ID定义
*/
enum {
  MICOLINK_MSG_ID_RANGE_SENSOR = 0x51, // 测距传感器
};

/*
    消息结构体定义
*/
typedef struct {
  uint8_t head;
  uint8_t dev_id;
  uint8_t sys_id;
  uint8_t msg_id;
  uint8_t seq;
  uint8_t len;
  uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
  uint8_t checksum;

  uint8_t status;
  uint8_t payload_cnt;
} MICOLINK_MSG_t;

/*
    数据负载定义
*/
#pragma pack(1)
// 测距传感器
typedef struct {
  uint32_t time_ms;     // 系统时间 ms
  uint32_t distance;    // 距离(mm) 最小值为10，0表示数据不可用
  uint8_t strength;     // 信号强度
  uint8_t precision;    // 精度
  uint8_t tof_status;   // 状态
  uint8_t reserved1;    // 预留
  int16_t flow_vel_x;   // 光流速度x轴
  int16_t flow_vel_y;   // 光流速度y轴
  uint8_t flow_quality; // 光流质量
  uint8_t flow_status;  // 光流状态
  uint16_t reserved2;   // 预留
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack()

```

```c++
// mtf01.c
#include "mtf01.h"

/*
说明： 用户使用micolink_decode作为串口数据处理函数即可

距离有效值最小为10(mm),为0说明此时距离值不可用
光流速度值单位：cm/s@1m
飞控中只需要将光流速度值*高度，即可得到真实水平位移速度
计算公式：实际速度(cm/s)=光流速度*高度(m)
*/

bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data);

void micolink_decode(uint8_t data) {
  static MICOLINK_MSG_t msg;

  if (micolink_parse_char(&msg, data) == false)
    return;

  switch (msg.msg_id) {
  case MICOLINK_MSG_ID_RANGE_SENSOR: {
    MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
    memcpy(&payload, msg.payload, msg.len);

    /*
        此处可获取传感器数据:

        距离        = payload.distance;
        强度        = payload.strength;
        精度        = payload.precision;
        距离状态    = payload.tof_status;
        光流速度x轴 = payload.flow_vel_x;
        光流速度y轴 = payload.flow_vel_y;
        光流质量    = payload.flow_quality;
        光流状态    = payload.flow_status;
    */
    break;
  }

  default:
    break;
  }
}

bool micolink_check_sum(MICOLINK_MSG_t *msg) {
  uint8_t length = msg->len + 6;
  uint8_t temp[MICOLINK_MAX_LEN];
  uint8_t checksum = 0;

  memcpy(temp, msg, length);

  for (uint8_t i = 0; i < length; i++) {
    checksum += temp[i];
  }

  if (checksum == msg->checksum)
    return true;
  else
    return false;
}

bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data) {
  switch (msg->status) {
  case 0: // 帧头
    if (data == MICOLINK_MSG_HEAD) {
      msg->head = data;
      msg->status++;
    }
    break;

  case 1: // 设备ID
    msg->dev_id = data;
    msg->status++;
    break;

  case 2: // 系统ID
    msg->sys_id = data;
    msg->status++;
    break;

  case 3: // 消息ID
    msg->msg_id = data;
    msg->status++;
    break;

  case 4: // 包序列
    msg->seq = data;
    msg->status++;
    break;

  case 5: // 负载长度
    msg->len = data;
    if (msg->len == 0)
      msg->status += 2;
    else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN)
      msg->status = 0;
    else
      msg->status++;
    break;

  case 6: // 数据负载接收
    msg->payload[msg->payload_cnt++] = data;
    if (msg->payload_cnt == msg->len) {
      msg->payload_cnt = 0;
      msg->status++;
    }
    break;

  case 7: // 帧校验
    msg->checksum = data;
    msg->status = 0;
    if (micolink_check_sum(msg)) {
      return true;
    }

  default:
    msg->status = 0;
    msg->payload_cnt = 0;
    break;
  }

  return false;
}

```

推测读取光流速度后直接减去陀螺仪获取的速度然后赋给ek3进行后续的数据融合即可

### Ardupilot官方文档中的校准传感器

见参考资料[6], 在文档中发现一出校准光流传感器的操作, 其中提到`FLOW_FXSCALER`, `FLOW_FYSCALER`两个参数用于把光流速度与imu速度调整到相近波形, 尚未经过测试这个参数的用处

![20230427141](https://i.imgur.com/nnqG9YQ.png)

![20230427143](https://i.imgur.com/pNj7bEZ.png)

## 参考资料

\[1] [光流定点若干问题分析_权盛光流 无法定高_engineerlixl的博客-CSDN博客](https://blog.csdn.net/u010720661/article/details/54017167)

[2] [Ardupilot光流代码分析 - CodeAntenna](https://codeantenna.com/a/uJiyzrXtJL)

[3] [PX4FLOW光流模块DIY（含部分代码讲解）_xian_z的博客-CSDN博客](https://blog.csdn.net/xian_z/article/details/76947952)

[4] [无人机—加速度计与光流数据融合_光流融合_经纬的无疆的博客-CSDN博客](https://blog.csdn.net/awujiang/article/details/91474998)

[5] [无人机飞控之光流知识小结（包含LK算法的讲解，across写的） | 码农家园 (codenong.com)](https://www.codenong.com/cs107096247/)

[6] [Optical Flow Sensor Testing and Setup — Copter documentation (ardupilot.org)](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html)