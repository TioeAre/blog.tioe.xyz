---
title: APM光流传感器解析
description:
date: 2023-05-14T16:14:45+08:00
categories:
  - Multi-Sensor
keyword:
  - OpticalFlow
tags:
  - OpticalFlow
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/77816288_p0_master1200.jpg
toc: true
math: true
comments: true
hidden: false
draft: false
---



## libraries/AP_OpticalFlow/AP_OpticalFlow.cpp

主要定义了`init()`, `handle_msp()`, `update()`, `start_calibration()`等几个函数

1.   `init()`

     主要用于接收光流传感器的类型并定义backend, backend是`libraries/AP_OpticalFlow/AP_OpticalFlow_Backend.h`中定义的OpticalFlow_backend基类对象, 派生出了PX4Flow, MAVLINK等几个具体传感器类型的派生类, 每个派生中重载了这个类的`update(), handle_msg()`函数, 分别用于获取光流数据和更新

2.   `update()`

     函数中主要调用了backend->update()的函数, 之后_calibrator->update()校准函数, _calibrator定义于start_calibration()中

     ```c++
     if (_calibrator != nullptr) {
         if (_calibrator->update()) {
             // apply new calibration values
             const Vector2f new_scaling = _calibrator->get_scalars();
             const float flow_scalerx_as_multiplier = (1.0 + (_flowScalerX * 0.001)) * new_scaling.x;
             const float flow_scalery_as_multiplier = (1.0 + (_flowScalerY * 0.001)) * new_scaling.y;
             _flowScalerX.set_and_save_ifchanged((flow_scalerx_as_multiplier - 1.0) * 1000.0);
             _flowScalerY.set_and_save_ifchanged((flow_scalery_as_multiplier - 1.0) * 1000.0);
             _flowScalerX.notify();
             _flowScalerY.notify();
             GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlowCal: FLOW_FXSCALER=%d, FLOW_FYSCALER=%d", (int)_flowScalerX, (int)_flowScalerY);
         }
     }
     ```

     _flowScalerX, _flowScalerY是地面站设置中的缩放参数. 这段代码用于更新缩放参数并发送给GCS

3.   `start_calibration()`

     ```c++
     if (_calibrator == nullptr) {
         _calibrator = new AP_OpticalFlow_Calibrator();
         if (_calibrator == nullptr) {
             GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "FlowCal: failed to start");
             return;
         }
     }
     if (_calibrator != nullptr) {
         _calibrator->start();
     }
     ```

     用于给_calibrator赋值

## libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp

主要派生了`OpticalFlow_backend`类, 重载了获取数据和更新用的`handle_msg() update()`函数, 新增了几个用于计算的变量

1.   `handle_msg()`

     ```c++
     mavlink_msg_optical_flow_decode(&msg, &packet);
     latest_frame_us = AP_HAL::micros64();
     flow_sum.x += packet.flow_x;
     flow_sum.y += packet.flow_y;
     quality_sum += packet.quality;
     count++;
     ```

     从飞控读取数据后将这段时间的光流数据累加起来, 后续update时用的光流速度实际上是i这段时间的平均值

2.   `update()`

     ```c++
     if (gyro_sum_count < 1000) {
             const Vector3f& gyro = AP::ahrs().get_gyro();
             gyro_sum.x += gyro.x;
             gyro_sum.y += gyro.y;
             gyro_sum_count++;
     }
     if (is_positive(dt) && (dt < OPTFLOW_MAV_TIMEOUT_SEC)) {
         // calculate flow values
         const float flow_scale_factor_x = 1.0f + 0.001f * _flowScaler().x;
         const float flow_scale_factor_y = 1.0f + 0.001f * _flowScaler().y;
         // copy flow rates to state structure
         state.flowRate = { ((float)flow_sum.x / count) * flow_scale_factor_x * dt,
                            ((float)flow_sum.y / count) * flow_scale_factor_y * dt };
         // copy average body rate to state structure
         state.bodyRate = { gyro_sum.x / gyro_sum_count, gyro_sum.y / gyro_sum_count };
         // we only apply yaw to flowRate as body rate comes from AHRS
         _applyYaw(state.flowRate);
     } else {
         // first frame received in some time so cannot calculate flow values
         state.flowRate.zero();
         state.bodyRate.zero();
     }
     ```

     首先读取了imu的数据然后也做了平均数, 光流中的`flowRate`就是光流速度平均值乘以缩放系数, `bodyRate`是imu的平均值

## libraries/AP_OpticalFlow/AP_OpticalFlow_Calibrator.cpp

主要用于计算最佳的缩放比例

```c++
struct sample_t {
        float flow_rate;	// 光流速度
        float body_rate;	// imu速度
        float los_pred;	//ekf估计的在车辆运动方向的光流速度的分量
    };
```

1.   `update()`

     ```c++
     if (_cal_state == CalState::RUNNING) {
         uint32_t now_ms = AP_HAL::millis();
         uint32_t timestamp_ms;
         Vector2f flow_rate, body_rate, los_pred;
         if (AP::ahrs().getOptFlowSample(timestamp_ms, flow_rate, body_rate, los_pred)) {
             add_sample(timestamp_ms, flow_rate, body_rate, los_pred);
             // while collecting samples display percentage complete
             if (now_ms - _last_report_ms > AP_OPTICALFLOW_CAL_STATUSINTERVAL_SEC * 1000UL) {
                 _last_report_ms = now_ms;
                 GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s x:%d%% y:%d%%",
                                 prefix_str,
                                 (int)((_cal_data[0].num_samples * 100.0 / AP_OPTICALFLOW_CAL_MAX_SAMPLES)),
                                 (int)((_cal_data[1].num_samples * 100.0 / AP_OPTICALFLOW_CAL_MAX_SAMPLES)));
             }
             // advance state once sample buffers are full
             if (sample_buffers_full()) {
                 _cal_state = CalState::READY_TO_CALIBRATE;
                 GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s samples collected", prefix_str);
             }
         }
         // check for timeout
         if (now_ms - _start_time_ms > AP_OPTICALFLOW_CAL_TIMEOUT_SEC * 1000UL) {
             GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s timeout", prefix_str);
             _cal_state = CalState::FAILED;
         }
     }
     // start calibration
     if (_cal_state == CalState::READY_TO_CALIBRATE) {
         // run calibration and mark failure or success
         if (run_calibration()) {
             _cal_state = CalState::SUCCESS;
             return true;
         } else {
             _cal_state = CalState::FAILED;
         }
     }
     ```

     先从AHRS中获取时间辍, 光流的速度和机体imu的速度, 然后运行run_calibration()进行校准

2.   `run_calibration()`

     ```c++
     // calculate total absolute residual from all samples
     float total_abs_residual = 0;
     for (uint8_t i = 0; i < num_samples; i++) {
         const sample_t& samplei = _cal_data[axis].samples[i];
         total_abs_residual += fabsf(calc_sample_residual(samplei, 1.0));
     }
     // 此时计算的是系数为1, 得到的也就是实际上的光流速度


     // 先计算残差和
     // 如果残差和为零的话就返回1, 说明校准成功
     // for each sample calculate the residual and scalar that best reduces the residual
     float best_scalar_total = 0;
     for (uint8_t i = 0; i < num_samples; i++) {
         float sample_best_scalar;
         const sample_t& samplei = _cal_data[axis].samples[i];
         if (!calc_sample_best_scalar(samplei, sample_best_scalar)) {
             // failed to find the best scalar for a single sample
             // this should never happen because of checks when capturing samples
             GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s failed because of zero flow rate", prefix_str);
             INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
             return false;
         }
         const float sample_residual = calc_sample_residual(samplei, 1.0);
         best_scalar_total += sample_best_scalar * fabsf(sample_residual) / total_abs_residual;
     	// 计算的是光流补偿量除以光流速度的平均值, 即系数的平均值
     }
     // 最佳残差时的系数, 判断光流速度是否存在零, 如果存在零的话就返回false
     // 当计算出的最佳缩放系数的均方残差, 当均方小于系数为1计算出的均方残差时返回true (不太理解, 只要系数小于1的话均方肯定也是比他小的呀)
     // 返回的缩放系数为best_scalar_total
     ```

3.   `calc_sample_residual()`

     ```c++
     float AP_OpticalFlow_Calibrator::calc_sample_residual(const sample_t& sample, float scalar) const
     {
         return (sample.body_rate + ((sample.flow_rate * scalar) - sample.los_pred));
     }
     ```

     计算的残差为$imu速度+(光流速度*缩放系数-EKF估计的光流速度在车辆运动方向的分量)$, (等于0时说明系数最佳)

4.   `calc_sample_best_scalar()`

     ```c++
     {
         // if sample's flow_rate is zero scalar has no effect
         // this should never happen because samples should have been checked before being added
         if (is_zero(sample.flow_rate)) {
             return false;
         }
         best_scalar = (sample.los_pred - sample.body_rate) / sample.flow_rate;
         return true;
     }
     ```

     $最佳系数=(EKF估计的光流速度在运动方向的分量-imu速度)/光流速度$
