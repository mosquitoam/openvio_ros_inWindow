# OPENVIO_ROS

## OPENVIO ROS module  

## **OPENVIO线上文档请点击下面链接** 

[OPENVIO线上文档](http://guanglundz.com/openvio)  

## 简介

OPENVIO 一款脱胎于OPENMV的智能摄像头。

在OPENMV硬件基础上增加了USB2.0芯片(USB3315）和IMU芯片（ICM20948），除却兼容OPENMV固件外，还可以将摄像头原始图像（未压缩图像）和IMU九轴数据高速传输至PC，可以作为SLAM单目IMU方案研究的低廉传感器方案（如港科大的VINS-MONO）.多种接口方便扩展更多功能，比如扩展超声波或激光模块后作为PX4光流模块使用（暂未实现 还在研发中）。

## 源码和资料

[OPENVIO源码](https://gitee.com/guanglunking/OPENVIO_BOARD)【开发环境：Keil5】  

[OPENVIO PC上位机](https://gitee.com/guanglunking/OPENVIO_PC)【开发环境：QT5.6.0 qt-opensource-windows-x86-mingw492-5.6.0】  

[OPENVIO ROS源码](https://gitee.com/guanglunking/OPENVIO_ROS)

[淘宝店铺](https://item.taobao.com/item.htm?id=615919130291)  

## 使用 

![VINS-MONO](https://images.gitee.com/uploads/images/2020/0804/130211_b8e933c0_683968.png "Screenshot from 2020-08-04 12-59-07.png")
#### HOW TO RUN

```
source ~/catkin_ws/devel/setup.bash
roscore
roslaunch vins_estimator vins_rviz.launch
roslaunch vins_estimator openvio.launch
rosrun openvio openvio_ros
```

#### Kalibr

* <=90
kalibr_calibrate_cameras --target april.yaml --bag 2020-08-03-14-14-48.bag --models pinhole-radtan --topics /cam0/image_raw

* >90
kalibr_calibrate_cameras --target april.yaml --bag 2020-08-03-14-14-48.bag --models omni-radtan --topics /cam0/image_raw

kalibr_calibrate_imu_camera --target april.yaml --cam camchain-2020-08-03-14-14-48.yaml --imu imu.yaml --bag 2020-08-03-14-14-48.bag