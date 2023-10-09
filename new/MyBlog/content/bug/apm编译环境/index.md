---
title: ardupilot编译依赖项安装失败
date: 2023-04-21T12:36:00+08:00
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/72581865_p0_master1200.jpg
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





前两天下载了ardupilot的源码，在配置编译环境的时候用了官方提供的`Tools/environment_install/install-prereqs-ubuntu.sh`，结果好多报错。

然后我去[pkgs](https://pkgs.org/)一个一个找apt安装失败的包，然后手动下载安装，遇到依赖问题就继续重新安装，到最后虽然成功安装好了，我一重启发现安装的时候好像给我桌面环境什么的自动卸掉了，想要`sudo apt install ubuntu-desktop`按回来，但是一直失败，提示我找不到这个包。我又wget下载了一个deb的包想用dpkg安装看看到底哪里有问题，发现确实是新安装的依赖的问题。

最后换了一下源，我之前一直用的初始的main源，现在换清华源了之后一下就好了，自动把我的所有依赖都更新成适合的，成功apt安装回桌面

```bash
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-security main restricted universe multiverse
```

