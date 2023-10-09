---
title: gazebo仿真环境下停机坪的识别
description: c++通过opencv识别仿真环境中的停机坪
date: 2023-03-16T16:14:45+08:00
categories:
  - OpenCV
keyword:
  - 停机坪
  - c++
  - OpenCV
tags:
  - c++
  - OpenCV
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/103449092_p0_master1200.jpg
toc: true
math: true
comments: true
hidden: false
draft: false
---

# OpenCV识别停机坪

整体算法目标为能够在真机通过`intel D435i`深度相机与下视的工业相机完成无人机对周边环境的感知, 并建立起稠密地图. 计划目标为先在方针环境中进行算法编写与验证. 然后再部署到真机中进行调整和测试. 本周的主要完成的在仿真环境中对停机坪的识别, 并添加`ros`支持

停机坪示意图如下

![iDBArx](https://i.imgur.com/ZsU1Wny.png)

## 停机坪识别概述

### 工程目录


```bash
identifyApron
├── bug.md	#记录程序编写过程中遇到的bug及解决方法
├── CMakeLists.txt	#工程配置文件
├── include	#存放头文件
│   ├── getImage.h
│   └── identify.h
├── media	#存放测试或程序运行用到的图像文件
│   ├── E.jpg
│   ├── image.png
│   ├── N.jpg
│   ├── S.jpg
│   └── W.jpg
├── package.xml	#ros依赖项配置
└── src	#存放源文件
    ├── getImage.cpp
    ├── identify.cpp
    └── main.cpp
```

本项目主要有`getImage`和`identify`两个文件, 其中`getimage`用于`ros`的订阅和发布图像节点. `identify`用于识别停机坪

### 配置文件

#### `CMakeLists.txt`

`cmake`中主要需要包含有`eigen`和`opencv`这两个库, 可以选用`ros`自带的`opencv4.2`然后通过`ros`安装`eigen`, 在这里我选用了之前本地安装过的`eigen3.4`和`opencv4.5.5`版本

为能够使用本地版本, 需要在`CMakeLists.txt`中添加以下配置

```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(cv_bridge_DIR "/home/${your_username}/cvbridge_build_ws/devel/share/cv_bridge/cmake")
set(OpenCV_DIR /usr/local/share/opencv4)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/home/${your_path_to_eigen}/eigen-3.4.0/Eigen)
```

首先指定`C++`版本为`c++14`, 以免某些特性无法通过编译. 其次需要自己手动重新编译一份`cv_bridge`, 原因是由于`ros`自带的`cv_bridge`匹配的是4.2版本, 直接用的话不和我电脑上的`opencv`适配, 会报类似于`memory out of range`的错误. 然后指定`cv_bridge_dir`为你重新编译的路径, 同时也需要手动指定`opencv`路径为你的安装路径

#### `package.xml`

`ros`包依赖中需要以下依赖

```xml
<buildtool_depend>catkin</buildtool_depend>
<build_depend>cv_bridge</build_depend>
<build_depend>image_transport</build_depend>
<build_depend>roscpp</build_depend>
<build_depend>sensor_msgs</build_depend>
<build_depend>std_msgs</build_depend>
<build_export_depend>cv_bridge</build_export_depend>
<build_export_depend>image_transport</build_export_depend>
<build_export_depend>roscpp</build_export_depend>
<build_export_depend>sensor_msgs</build_export_depend>
<build_export_depend>std_msgs</build_export_depend>
<exec_depend>cv_bridge</exec_depend>
<exec_depend>image_transport</exec_depend>
<exec_depend>roscpp</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

主要用于订阅和发布图像传输的节点, 自定义发布数据

## 算法实现

### `ros`部分

`ros`部分主要就是订阅和发布图像, 下面根据代码进行解释

#### getImage.h

```c++
//定义一个转换的类
class ImageDetector {
private:
    ros::NodeHandle nh_;    //定义ROS节点句柄
    image_transport::ImageTransport it_;    //定义一个image_transport实例,用来发布和订阅ROS系统的图像
    image_transport::Subscriber image_sub_; //定义ROS图象接收器 ,订阅主题的变量
    image_transport::Publisher image_pub_;  //定义ROS图象发布器 ,发布主题的变量
    identifyApron iden;
public:
    ImageDetector();

    ~ImageDetector() {}

    void convert_callback(const sensor_msgs::ImageConstPtr &msg);   //回调函数包含识别和发布识别后的图片
    void advertiseDone();
};
```

首先头文件中定义一个用于将`opencv`的`Mat`类型的图像转化为`ros`中的数据格式, 在后面的源文件中有相关介绍, 头文件主要定义了用到的变量

定义了`ros`相关节点句柄和用于识别的`identifyApron`对象, 两个函数分别用于接受图像并转化和发布图像. 其中接受到图像后在`convert_callback`函数中调用`identifyApron`的相关接口进行识别

#### getImage.cpp

源文件定义了类的构造函数和其他两个函数的写法

```c++
ImageDetector::ImageDetector() : it_(nh_)   //构造函数
{
    // /iris_0/camera/image_raw
    image_sub_ = it_.subscribe("/iris_0/stereo_camera/left/image_raw", 1, &ImageDetector::convert_callback,
                               this);   //定义图象接受器
    image_pub_ = it_.advertise("/image_detector/output_video", 1);  //定义图象发布器
}
```

构造函数接受头文件中定义的句柄并定义订阅和发布使用的节点

```c++
void ImageDetector::convert_callback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    iden.detector(cv_ptr->image);
    advertiseDone();
}

void ImageDetector::advertiseDone() {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", iden.origin).toImageMsg();
    image_pub_.publish(msg);
}
```

转化函数调用`cv_bridge`对`Mat`类型进行转化, 然后调用`identifyApron`中的`detector()`进行图像的识别, 最后调用发布识别后的图像的函数

### 识别部分

接下来是工程最主要的识别部分

####  `identiyf.h`

```c++
class identifyApron {
public:
    cv::Mat origin;
    void detector(const cv::Mat &frame);
    identifyApron();

private:
    cv::Scalar scalarLow = cv::Scalar(26, 43, 100); //hsv分离最小阈值
    cv::Scalar scalarHigh = cv::Scalar(35, 255, 255);   //hsv最大阈值
    std::vector<std::vector<cv::Point>> allContours;   //所有轮廓
    std::vector<cv::Vec4i> hiera;   //轮廓的层级关系
    int mediaB = 3; //中值滤波窗口
    int openEle = 2;   //开运算尺寸
    int closeEle = 3; //闭运算尺寸
    int roughDilate = 52;
    std::vector<cv::Vec3f> circles; //霍夫圆的轮廓点
    std::vector<cv::RotatedRect> alRect;    //所有旋转矩形
    std::vector<cv::RotatedRect> charRect;  //字母的旋转矩形
    cv::Point2f charRectPts[4];
    std::vector<std::pair<std::string , int>> direction = {{"E", 0},    //东西南北
                                                           {"W", 1},
                                                           {"S", 2},
                                                           {"N", 3}};
    std::vector<cv::Mat> chars;
    std::vector<cv::Mat> template_pictures;
    std::vector<Eigen::Matrix<int, 50, 50>> char_matrix;
//    int mat_size = 50;
    int matchMat(cv::Mat src);
};
```

头文件主要定义了函数接口和识别过程中用到的参数

`matchMat()`函数主要用于模板匹配, 为了识别出停机坪四周表示方向的字母

#### `identify.cpp`

识别文件中首先定义了几个后续用到的函数

```c++
void restore_Rect(cv::Point2f pts[4], cv::Point2f points[4]);   //还原外接矩形
float dis(cv::Point2f pts1, cv::Point2f pts2);  //计算两点距离
void drawRect(cv::RotatedRect rect, const cv::Mat &image, const cv::Scalar scal);   //画出旋转矩形
cv::Mat getRectROI(cv::RotatedRect rect, const cv::Mat &bilfilter);  //得到旋转矩形的roi
```

这几个函数的用途如注释中写的, 用于还原外接矩形四个点的顺序, 计算像素点之间的距离等

主要的`detector()`函数首先将获取到的图像进行预处理

```c++
    origin = frame.clone();
    cv::Mat hsv, filter, bilfilter, masked, cannied, round;
    cv::cvtColor(origin, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, scalarLow, scalarHigh, masked);
    cv::medianBlur(masked, filter, mediaB);
    cv::bilateralFilter(filter, bilfilter, 5, 10, 3);
    cv::Mat struElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(openEle, openEle), cv::Point(-1, -1));
    cv::morphologyEx(bilfilter, bilfilter, cv::MORPH_OPEN, struElement);
    struElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closeEle, closeEle), cv::Point(-1, -1));
    cv::morphologyEx(bilfilter, bilfilter, cv::MORPH_CLOSE, struElement);
```

先复制一份图像以免处理图像时受到其他意外影响本函数的运行

然后将`bgr`格式的图像转化到`hsv`色彩空间中, 这是由于`bgr`表示的图像像素点会受到光照影响从而改变颜色范围, 而`hsv`将图像表示为色相, 饱和度, 亮度三部分. 将亮度和色相分割开, 有效避免了阴影与图像颜色的影响

![iDBfJw](https://i.imgur.com/wQJRapK.png)

可以看到阈值化后已经隔除了红色墙壁和绿色部分对识别的影响, 但是图像中仔细观察会发现还有白色部分里有一些小黑点, 会影响到后续的识别. 因此需要再进行膨胀, 腐蚀, 滤波等处理

```c++
    cv::medianBlur(masked, filter, mediaB);
    cv::bilateralFilter(filter, bilfilter, 5, 10, 3);
    cv::Mat struElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(openEle, openEle), cv::Point(-1, -1));
    cv::morphologyEx(bilfilter, bilfilter, cv::MORPH_OPEN, struElement);
    struElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closeEle, closeEle), cv::Point(-1, -1));
    cv::morphologyEx(bilfilter, bilfilter, cv::MORPH_CLOSE, struElement);
    cv::Canny(round, cannied, 10, 250, 5);
```

首先中值滤波取出可能存在的椒盐噪点, 然后双边滤波和闭运算, 处理完成后再进行边缘检测用于霍夫圆监测

边缘检测效果如下

![iDBVBz](https://i.imgur.com/tV5CaOk.png)

在上上一步中获取到的图像在闭运算后可以用于检测字母, 其主要思想为

1.   查找图像中的所有轮廓并列出轮廓的层级关系, 很明显需要检测的字母轮廓是没有子轮廓和父轮廓的, 但是停机坪的圆弧同样没有父子轮廓
2.   但观察很容易可以看出字母的轮廓的最小外接矩形近似于正方形, 而圆弧的最小外接矩形为长宽差异很大的长方形. 因此可以先求出外接矩形的长宽比然后筛选得出字母的轮廓
3.   得到轮廓后需要进行识别, 这里识别字母可以用神经网络的方法进行训练(如`knn`, `lenet`等). 但是停机坪字母固定并且只有四个, 没有必要训练那么多, 因此我采用了类似模板匹配的方法
4.   首先制作一个字母灰度图的模板, 然后将`2`中获得的最小外接矩形旋转到正方向, 进行缩放为模板大小, 然后再循环`90`, `190`, `270`三种度数与模板做矩阵减法并求和, 此处用到了`eigen`将`Mat`转化为`Matrix`然后加快求解速度

```c++
    ///检测字母
    cv::findContours(bilfilter, allContours, hiera, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);   //查找轮廓
//    cv::drawContours(origin, allContours, -1, cv::Scalar(0, 0, 255), 1);
    int i = 0;
    for (const auto &itcHier: hiera) {
        if (itcHier[2] == -1 && itcHier[3] == -1) {
            alRect.push_back(cv::minAreaRect(allContours[i]));
        }//没有子轮廓和父轮廓可能是目标圆弧或字母
        ++i;
    }
    cv::Point2f pts[4], pts1[4];
    i = 0;
    for (const auto &rect: alRect) {
        rect.points(pts);
        restore_Rect(pts, pts1);
        float width = dis(pts1[0], pts1[3]);
        float height = dis(pts1[0], pts1[1]);
        if ((width / height) > 0.8 && (width / height) < 1.2) {   ///调参, 去除圆弧
            charRectPts[i].x = pts1[0].x;
            charRectPts[i].y = pts1[0].y;
            ++i;
            charRect.push_back(rect);
            drawRect(rect, origin, cv::Scalar(0, 255, 100));
            chars.push_back(getRectROI(rect, bilfilter));
        }
    }
    if(chars.size() != 0){
        for (i = 0; i < chars.size(); i++) {
            int number = matchMat(chars[i]);    //std::to_string(number)
            cv::putText(origin, direction[number].first, cv::Point((int) charRectPts[i].x, (int) charRectPts[i].y),
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(50, 100, 255), 2, cv::LINE_AA);
        }
    }
```

模板匹配实现如下

```c++
int identifyApron::matchMat(cv::Mat src) {
    int angle = 0;
    if (false) {
        label:
        if (angle > 270) return 4;
        cv::Size dst_sz(src.cols, src.rows);
        cv::Point2f center(static_cast<float>(src.cols / 2.), static_cast<float>(src.rows / 2.));
        angle += 90;
        cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);
        cv::warpAffine(src, src, rot_mat, dst_sz, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    }
    int i = 0;
    for (const Eigen::Matrix<int, 50, 50> &matrix: char_matrix) {
        Eigen::Matrix<int, 50, 50> src_matrix;
        cv::cv2eigen(src, src_matrix);
        Eigen::ArrayXXi result = (matrix - src_matrix).array().abs();
        long re = (long) result.sum();
        if (re < 150000) {
            std::cout << result;
            return i;
        } else if (i >= 3) {
            goto label;
        }
        ++i;
    }
    return 4;
}
```

该函数接受待匹配字母, 返回头文件中定义的`std::vector<std::pair<std::string , int>> direction`, 用<字母, 数字>表示方向

最终识别效果如下

![iDB4Aa](https://i.imgur.com/VCq1ocz.png)

很好的识别出了四个方向的字母, 并拟合无人机的大概降落位置
