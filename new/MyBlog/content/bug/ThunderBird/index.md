---
title: ThunderrBird同步
date: 2023-04-06T00:00:00+08:00
image: https://raw.githubusercontent.com/TioeAre/imageHost/main/pictures/74923161_p0_master1200.jpg
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

ThunderrBird是ubuntu下自带的一个很好用的邮件软件，但是实际使用中总会遇到一些同步失败的情况，之前在windows下用outlook同步一些邮件的时候也会时常发生同步失败，并且由于没有那么经常用邮箱。而我的三星手机自带的邮件app又从未发生过同步失败，并没有很影响我的正常生活，所以之前也就一直没有管他。今天刚好又遇到了这个问题，就查了一下准备彻底解决他。

## gmail无法同步

即使挂了代理，也经常会出现同步失败，登陆验证不了的情况。

最后查过之后发现是账户设置的问题，thunderbird会自动把gmail的登陆验证模式改为OAuth2

![iIzQXF](https://i.imgur.com/snSbaso.png)

需要在Saved Passwords...中将imap，smtp开头的gamil密码删掉，只保留oauth开头的密码

![iIFfcL](https://i.imgur.com/ZoB4FrQ.png)

## outlook同步失败

此外还发现一个比较奇怪的现象，outlook在开代理之后不论是登陆还是同步都会失败，代理关掉就正常了。按理来说微软的outlook国外ip不应该不能访问的呀。

无奈，只能手动取消掉outlook的代理

![iIFpvN](https://i.imgur.com/ddOV8ON.png)

## 同步outlook日历

安装这三个插件并在TbSync中登陆outlook账号即可

![iIFdpk](https://i.imgur.com/PwPAoo2.png)