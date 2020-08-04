# OPENVIO_ROS

OPENVIO ROS module  

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