# 点云建图

该ros包需要配合我们修改过的ORB-SLAM2进行使用（使用分支V1.0.0），由ORB-SLAM2或者其他的位姿估计节点输出相机的位姿，该节点接受彩色图与深度图生成点云，根据位姿估计器输出的相机位姿Tcw对局部点云进行拼接。

详细的教程请参考csdn博客系列教程：[ORB-SLAM2 在线构建稠密点云](https://blog.csdn.net/crp997576280/article/details/88899163)

![Distribute SLAM](https://github.com/RuPingCen/pointcloud_mapping/raw/v1.0.0/images/image-1.png)

使用说明

```
git clone https://github.com/RuPingCen/pointcloud_mapping.git 　

roslaunch pointcloud_mapping pointcloud_mapping.launch
```
# Useage of Distribute SLAM

1. clone the pose estimater node  (This is an extended ORB-SLAM2 system)

```
cd catkin_ws/src

git clone https://github.com/RuPingCen/ORB-SLAM2.git 

cd ORB_SLAM2

./build.sh

./build_ros.sh

```
 
2. clone the mapping node and elas lib

note: the elas-lib is used for calculate the disparity from the stereo camera

```
cd catkin_ws/src

git clone https://github.com/RuPingCen/pointcloud_mapping.git 

git clone https://github.com/jeffdelmerico/cyphy-elas-ros.git

```

3. build workspace
```
cd catkin_ws

catkin_make

```



## RGBD Model

1. download the TUM rosbag form the [website](https://vision.in.tum.de/data/datasets/rgbd-dataset), then open a terminal and play the rosbag

```
rosbag play rgbd_dataset_freiburg1_room.bag
```

2. launch the pose estimator
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/YOUR-PATH/ORB_SLAM2/Examples/ROS

rosrun ORB_SLAM2 astra Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/TUM1_ROSbag.yaml

```
3. launch the mapping node with launch file

```
cd catkin_ws/src

source devel/setup.bash

roslaunch pointcloud_mapping tum1.launch

```
![STEREO Model Distribute SLAM](https://github.com/RuPingCen/pointcloud_mapping/raw/v1.0.0/images/image-3.png)


## STEREO Model
1. download the KITTI images form the [website](http://www.cvlibs.net/datasets/kitti/eval_odometry.php), then the python script is used to convert the images to rosbag.

```
roslaunch publish_image_datasets publish_kitti.launch 
```

2. launch the pose estimator
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/YOUR-PATH/ORB_SLAM2/Examples/ROS

rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/KITTI00-02.yaml false

```
3. launch the mapping node with launch file

```
cd catkin_ws/src

source devel/setup.bash

roslaunch elas_ros kitti_no_rviz.launch

roslaunch pointcloud_mapping kitti.launch

```
 

This is a demo on KITTI dataset [demo STEREO](https://youtu.be/T7gtojA4-os)


![STEREO Model Distribute SLAM](https://github.com/RuPingCen/pointcloud_mapping/raw/v1.0.0/images/image-2.png)


## Mick robot

The Mick is a home-made diff-wheeled car, equipped with ZED binocular camera.

This is a demo on Mick [demo STEREO](https://www.youtube.com/watch?v=OMDPEMi6yoA)






