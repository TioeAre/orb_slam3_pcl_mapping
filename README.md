# orb slam3稠密建图

## 项目介绍

### 开发环境

ubuntu 20.04

ros noetic

OpenCV 4.5.5

pcl 1.13

```bash
orb_slam3_pcl_mapping
├── CMakeLists.txt
├── package.xml
├── README.md
└── src
    ├── PointCloudMapper.cc
    ├── PointCloudMapper.h
    └── pointcloud_mapping.cpp
```

主要参照以下项目进行修改适配orb slam3与d455，并增加点云的回环

https://blog.csdn.net/crp997576280/article/details/104220926

https://github.com/xiaobainixi/ORB-SLAM2_RGBD_DENSE_MAP

## 代码简介

### PointCloudMapper.cc

头文件中主要定义了相关变量和函数

### void viewer();

唯一一个由外部调用的函数，主要用于点云的拼接和显示

```c++
KFUpdate = false;
{
    std::unique_lock<std::mutex> lck(keyframeMutex);
    N = mvGlobalPointCloudsPose.size();
    KFUpdate = mbKeyFrameUpdate;
    mbKeyFrameUpdate = false;
}
```

用于检测是否有新加入的关键帧

```c++
std::unique_lock<std::mutex> lock_loop(loopUpdateMutex);
PointCloud::Ptr tem_cloud1(new PointCloud());
tem_cloud1 = generatePointCloud(colorImgs.back(), depthImgs[colorImgs.size() - 1],
mvGlobalPointCloudsPose[colorImgs.size() - 1]);

if (tem_cloud1->empty())
	continue;
*globalMap += *tem_cloud1;
```

增加线程锁是为了避免与后面的回环造成数据冲突，每有一个新的关键帧加入时，将关键帧对应的点云和rgb图像以及此时相机的位姿进行转化，转化到世界坐标系下的彩色点云

### void PointCloudMapper::callback(...)

用于接受订阅消息的回调函数，接收到图像和位姿后将`geometry_msgs::PoseStamped::ConstPtr`的位姿转化为`Eigen::Isometry3f`类型，然后调用`insertKeyFrame(...)`将接收到的数据储存起来

当检测到回环是，不再接受某一关键帧的位姿信息，而是接收BA后的相机轨迹，再将每一帧的轨迹存储到`Eigen::Isometry3f`的`vector`中

### void PointCloudMapper::boolCallback()

检测到回环时调用的回调函数，主要用于重新计算点云，将关键帧的点云与BA后的位姿对应起来并拼接

## orb slam3修改部分

### ros_rgbd.cc

```c++
#include<iostream>
#include<chrono>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber {
public:
    ros::NodeHandle nh1;
    ros::Publisher pub_rgb, pub_depth, pub_tcw, pub_camerapath, pub_odom, pub_isLoop;
    size_t mcounter = 0;
    nav_msgs::Path camerapath;

    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM), nh1("~") {
        pub_rgb = nh1.advertise<sensor_msgs::Image>("RGB/Image", 1);
        pub_depth = nh1.advertise<sensor_msgs::Image>("Depth/Image", 1);
        pub_tcw = nh1.advertise<geometry_msgs::PoseStamped>("CameraPose", 1);
        pub_odom = nh1.advertise<nav_msgs::Odometry>("Odometry", 1);
        pub_camerapath = nh1.advertise<nav_msgs::Path>("Path", 1);
        pub_isLoop = nh1.advertise<std_msgs::Bool>("isLoop", 1);
    }

    // ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    ORB_SLAM3::System *mpSLAM;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RGBD");
    ros::start();

    if (argc != 3) {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
    ros::NodeHandle nh;
    ImageGrabber igb(&SLAM);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

tf::Pose trans_pose(Sophus::SE3f se3, bool& if_empty) {
    cv::Mat Tcw = (cv::Mat_<float>(4, 4) <<
                                         se3.matrix()(0, 0), se3.matrix()(0, 1), se3.matrix()(0, 2), se3.matrix()(0, 3),
            se3.matrix()(1, 0), se3.matrix()(1, 1), se3.matrix()(1, 2), se3.matrix()(1, 3),
            se3.matrix()(2, 0), se3.matrix()(2, 1), se3.matrix()(2, 2), se3.matrix()(2, 3),
            0.0f, 0.0f, 0.0f, 1.0f);
    if_empty = Tcw.empty();
    cv::Mat RWC = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tWC = Tcw.rowRange(0, 3).col(3);
    tf::Matrix3x3 M(RWC.at<float>(0, 0), RWC.at<float>(0, 1), RWC.at<float>(0, 2),
                    RWC.at<float>(1, 0), RWC.at<float>(1, 1), RWC.at<float>(1, 2),
                    RWC.at<float>(2, 0), RWC.at<float>(2, 1), RWC.at<float>(2, 2));
    tf::Vector3 V(tWC.at<float>(0), tWC.at<float>(1), tWC.at<float>(2));
    tf::Quaternion q;
    M.getRotation(q);
    tf::Pose tf_pose(q, V);
    double roll, pitch, yaw;
    M.getRPY(roll, pitch, yaw);
    if (roll == 0 || pitch == 0 || yaw == 0) if_empty = true;
    return tf_pose;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (cv_ptrRGB->image.empty() || cv_ptrD->image.empty()) return;
    Sophus::SE3f se3 = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    bool if_empty;
    tf::Pose tf_pose = trans_pose(mpSLAM->current_all_KF.back()->GetPose(), if_empty);
    if (!if_empty) {
        std_msgs::Header header;
        header.stamp = msgRGB->header.stamp;
        header.seq = msgRGB->header.seq;
        header.frame_id = "camera";
        //cout<<"depth type: "<< depth. type()<<endl;
        sensor_msgs::Image::ConstPtr rgb_msg = msgRGB;
        sensor_msgs::Image::ConstPtr depth_msg = msgD;

        geometry_msgs::PoseStamped tcw_msg;
        tcw_msg.header = header;
        tf::poseTFToMsg(tf_pose, tcw_msg.pose);

        camerapath.header = header;
        camerapath.poses.push_back(tcw_msg);
        std_msgs::Bool isLoop_msg;
        isLoop_msg.data = mpSLAM->is_loop;
        if (mpSLAM->is_loop) {
            pub_isLoop.publish(isLoop_msg);
            vector<geometry_msgs::PoseStamped> after_loop_poses;
            for(auto&& it : mpSLAM->current_all_KF){
                tf::Pose tf_pose = trans_pose(it->GetPose(), if_empty);
                it->mnId;
                if (if_empty) continue;
                geometry_msgs::PoseStamped tcw_msg;
                tcw_msg.header = header;
                tf::poseTFToMsg(tf_pose, tcw_msg.pose);
                after_loop_poses.push_back(tcw_msg);
                camerapath.poses.swap(after_loop_poses);
            }
        }
        pub_camerapath.publish(camerapath);
        if (mpSLAM->is_key_frame) {
            pub_tcw.publish(tcw_msg);
            pub_rgb.publish(rgb_msg);
            pub_depth.publish(depth_msg);
        }
    }
}

```

主要用于发布相关话题

### System.cc

```c++
Sophus::SE3f System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp,
                                   const vector<IMU::Point> &vImuMeas, string filename) {
    is_key_frame = mpTracker->is_key_frame;
    is_loop = false;
    is_loop = mpTracker->is_loop;
    current_all_KF = mpAtlas->GetAllKeyFrames();
    sort(current_all_KF.begin(),current_all_KF.end(),KeyFrame::lId);
    if (is_loop){
        mpTracker->is_loop = false; //本来应该加线程锁，但是应该几乎不会出现这种错误，除非算力低到5秒一帧
    }								//用于防止连续几帧中因为回环检测线程的休眠让程序重复再同一位置回环
    
}
```

主要增加了以上用于判断是否为关键帧，是否检测到回环的相关变量和判断，并获取此时所有的关键帧

### Tracking.cc

```c++
void Tracking::Track() {
    
	if (bNeedKF && (bOK || (mInsertKFsLost && mState == RECENTLY_LOST &&
                            (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                             mSensor == System::IMU_RGBD)))) {
        CreateNewKeyFrame();
        is_key_frame = true;
    } else {
        is_key_frame = false;
    }
}
```

主要在`Track()`中添加了判断是否为关键帧

### LoopClosing.cc

```c++
while(1){
     mpTracker->is_loop = false;
	...
	if (mbLoopDetected) {
        mpTracker->is_loop = true;
    }
}
```

同样添加用于判断是否检测到了回环

## 运行效果

![ibZAzc.png](https://i.328888.xyz/2023/04/03/ibZAzc.png)

使用intel relsense d455回绕一圈后构建的点云图

