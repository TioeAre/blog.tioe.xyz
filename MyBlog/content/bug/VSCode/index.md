---
title: vscode cmake find_package()找不到catkin
date: 2023-04-21T12:36:00+08:00
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/77816288_p0_master1200.jpg
categories:
  - bug
tags:
  - bug
toc: true
math: true
comments: true
hidden: false
draft: false
---



之前一直在用clion，但是clion占用内存太多了，打开有点慢，并且同时运行几项目的话有点卡，补全和提示之类的功能延迟会变得很高甚至就没有了。

然后想转到vscode只做为临时打开某个项目运行一下的编辑器，开始用了几天没遇到什么大问题，然后今天突然打开一个ros项目的时候，cmake报错找不到catkin。就很奇怪，不知道怎么回事，后来找到一个只能说是临时的解决方案，在`setting.json`中加一行

```json
"cmake.configureArgs": [
        "-DCMAKE_PREFIX_PATH=/opt/ros/noetic"
    ],
```

手动设置一下cmake的查找路径