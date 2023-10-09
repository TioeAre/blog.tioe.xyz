---
title: nice slam论文解析
description:
date: 2023-05-30T09:57:45+08:00
categories:
  - slam
keyword:
  - nice_slam
tags:
  - slam
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/72581865_p0_master1200.jpg
toc: true
math: true
comments: true
hidden: false
draft: false

---

![VN1u9P](https://i.imgur.com/vAVNKIp.jpg)

​	NICE-SLAM是一种利用神经隐式表达的稠密slam系统,主要维护了一个4层的feature grids的全局地图, 每个grid表示当前相机位姿下的局部的场景, 4个层级分别表示由粗到细的三个深度图和一个rgb层. 每个层级的每个grid储存32维的特征向量.

​	从右到左: 系统相当于一个场景渲染器。接收场景的特征网格和相机位姿，生成带有深度信息和色彩信息的RGB-D图像（深度信息处理为场景三维模型，色彩信息处理为贴图）；

​	从左往右: 输入采集的RGB-D图像流（ground truth），把右边渲染出的RGB-D估计值与坐边输入的RGB-D实际值比对，分别计算关于深度和色彩的损失函数，并且将损失函数沿着可微分渲染器反向传播。经过迭代，对特征网格和相机位姿进行优化（将损失函数降到最低），完成建图（Mapping）和定位（Tracking）

​	在建图过程中, NICE-SLAM使用如上图所示的架构, 首先得到一个先验的相机位姿, 然后在这个相机位姿下的不同层级的grid中按照相机朝向的ray进行随机采样一些点, 再用线性插值的方法得到每个grid中点的32维feature. 再用一个预训练好的decoder对这些先验的 feature点进行decode成深度图和彩色图. 再将decode出的深度图和彩色图与实际深度相机得到的深度图和彩色图进行对比做loss. 然后反过来去训练输入的feature上的点. 最终的到能够decode成正确深度图和彩色图的feature grid. 此时的feature grid就是slam建立起的稠密地图

​	NICE-SLAM建图的过程是tracking和mapping交替进行。Mapping的过程是以当前相机的RGB和depth作为真值，根据Feature grids渲染得到的RGB和depth作为预测值，构建几何或光度误差损失函数，同时对相机位姿和Feature grids中的feature进行优化。Tracking的过程就是基于Feature grids和估计的位姿渲染depth和rgb，通过和相机实际采集的depth和rgb对比，从而优化相机位姿。

​	NICE-SLAM的优势主要是与iMAP相比, iMAP是基于经典的NERF架构, 整个网络使用一个大的MLP来储存场景信息, 难以对大型场景进行细节建图, 并且在每次更新地图时都要对整个MLP的网络进行更新, 容易将已经学习到的细节覆盖掉造成场景的遗忘. 相比而言, NICE-SLAM使用 由粗到细的分场景表达, 对三个层级分别应用不同的预训练好的MLP. 并且能够允许对每个grid进行局部更新. 此时分辨率较低的coarse主要进行大场景的渲染, 因为分辨率较高, 所以能够允许网络对相机观测不到的场景也进行一个简单的预判, 大致填充没有观测到的区域. 分辨率最高的fine层级则负责对局部场景进行建模, 保证了网络渲染出的地图的精确度. 但是NICE-SLAM中对rgb信息的训练只用了一个MLP, 因此在回复每个体素的rgb时会存在一定的遗忘问题.

​	在网络架构中最右侧的黄色部分表示分层的特征网络, 其中红色的点表示空间中的样本点P, 与P相连的深蓝色的点表示三线性插值得到的几何参数表示为$\theta$. 此时这个样本点周围的函数场就表示为$\phi_\theta(p)$, 即深蓝色点与红色点的连线. 与每个特征网络对应的是各自的decode $f$, decode将样本点P和feature map中的值解码成体素的占用概率$o_p$, 针对分辨率最高的fine层级额外添加了一个独立的网络$\psi_\omega$用于表示rgb色彩信息对应decoder $g_\omega$, 在分辨率最高的层级中渲染出彩色的稠密地图

​	分辨率最低的corase层级仅用于估计没有观测到的场景中的缝隙$o^0_p=f^0(p,\phi^0_\theta(p))$

​	在渲染过程中, mid层级的decoder将输入样本点座标和feature grid计算出基础占用$o^1_p=f^1(p,\phi^1_\theta(p))$

​	fine层级将高分辨率的样本点和grid以及上一步mid层级得到的输出一同作为输入, $\Delta o^1_p=f^2(p,\phi^1_\theta(p),\phi^2_\theta(p))$, $o_p=o^1_p+\Delta o^1_p$

在色彩信息的渲染中, 使用$c_p=g_\omega(p, \psi _\omega(p))$

​	得到渲染输出后, 对于每条ray，在corase和fine两个层面的深度级别的深度，以及颜色可以被呈现为

$$
\hat{D}^c=\sum^{N}_{i=1}\omega^c_id_i,\  \hat{D}^f=\sum^{N}_{i=1}\omega^f_id_i,\  \hat{I}=\sum^{N}_{i=1}\omega^f_ic_i
$$

沿每条ray 的深度值

$$
\hat{D}^c_{var}=\sum^{N}_{i=1}\omega^c_i(\hat{D}^c-d_i)^2,\  \hat{D}^f_{var}=\sum^{N}_{i=1}\omega^f_i(\hat{D}^f-d_i)^2
$$

​	建图从当前帧和选定的关键帧中均匀采样总共M个像素. 然后采用分阶段的方式来最小化几何和光度损失. 几何损失只是corase或fine层次上观测值和预测深度之间的L1损失
$$
\iota^l_g=\frac{1}{M}\sum_{m=1}^{M}\begin{vmatrix} D_m-\hat{D}^l_m \end{vmatrix}
$$

​	对于M个采样像素，光度损失也是渲染和观察颜色值之间的L1损失
$$
\iota_p=\frac{1}{M}\sum_{m=1}^{M}\begin{vmatrix}I_m-\hat{I}^l_m\end{vmatrix}
$$

​	在第一阶段, 只优化中级特征网格$ϕ_θ^1$, 使用几何损失$L_g^f$, 然后共同优化中级和精细级$ϕ_θ^1$, $ϕ_θ^2$特征与相同的精细级深度损失$L_g^f$, 最后，进行局部BA, 共同优化所有层级的特征网格, 颜色解码器以及K个选定关键帧的相机外部参数$R_i, t_i$
$$
min_{\theta,\omega,\{R_i,t_i\}}(\iota^c_g+\iota^f_g+\lambda_p\iota_p)
$$
​	相机跟踪中对当前$M_t$的像素进行采样, 应用光度损失

$$
\iota_{g_mvar}=\frac{1}{M} \sum^{M_t}_{m=1} (\frac{\begin{vmatrix} D_m-\hat{D}^c_m \end{vmatrix}}{\sqrt{\hat{D}^c_{var}}}+\frac{\begin{vmatrix} D_m-\hat{D}^f_m \end{vmatrix}}{\sqrt{\hat{D}^f_{var}}})
$$

几何损失
$$
min_{R,t}(\iota_{g_mvar+\lambda_{pt}\iota_p})
$$

