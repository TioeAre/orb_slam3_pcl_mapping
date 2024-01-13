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

`注意`: 这个仓库本身可以作为单独的一个用作接受点云与相应位姿, 并可视化保存拼接后的点云的功能包, 并不一定需要嵌入到orb slam中

## 代码简介

### PointCloudMapper.cc

头文件中主要定义了相关变量和函数

### void viewer();

唯一一个由外部调用的函数，主要用于点云的拼接和显示，以及判断当前是否检测到回环

```c++
while (ros::ok()) {
    ros::spinOnce();
    //用于检测是否有关键帧加入
    KFUpdate = false;
    {
        std::unique_lock<std::mutex> lck(keyframeMutex);
        N = mvGlobalPointCloudsPose.size();
        KFUpdate = mbKeyFrameUpdate;
        mbKeyFrameUpdate = false;
    }
    //是否有关键帧加入或是否是回环模式
    if ((KFUpdate && N > lastKeyframeSize) || is_loop_for_remap) {
        std::unique_lock<std::mutex> lock_loop(loopUpdateMutex);
        //如果是回环的话根据BA后的位姿重新绘制点云
        if (is_loop_for_remap) {
            std::cout << RED << "detect loop!" << std::endl;
            std::cout << "mvGlobalPointCloudsPose size: " << mvGlobalPointCloudsPose.size() << std::endl;
            std::cout << "depthImgs size: " << depthImgs.size() << std::endl;
            std::cout << "colorImgs size: " << colorImgs.size() << std::endl;
            globalMap->clear();
            for (int i = 0; i < depthImgs.size(); i += 1) {
                tmp->clear();
                for (int m = 0; m < depthImgs[i].rows; m += 3) {
                    for (int n = 0; n < depthImgs[i].cols; n += 3) {
                        float d = depthImgs[i].ptr<float>(m)[n] / mDepthMapFactor;
                        if (d < 0 || d > max_distance) {
                            continue;
                        }
                        PointT p;
                        p.z = d;
                        p.x = (n - mcx) * p.z / mfx;
                        p.y = (m - mcy) * p.z / mfy;
                        p.r = colorImgs[i].data[m * colorImgs[i].step + n * colorImgs[i].channels()];
                        p.g = colorImgs[i].data[m * colorImgs[i].step + n * colorImgs[i].channels() + 1];
                        p.b = colorImgs[i].data[m * colorImgs[i].step + n * colorImgs[i].channels() + 2];
                        tmp->points.push_back(p);
                    }
                }
                cloud_voxel_tem->clear();
                tmp->is_dense = false;
                voxel.setInputCloud(tmp);
                voxel.setLeafSize(mresolution, mresolution, mresolution);
                voxel.filter(*cloud_voxel_tem);
                cloud1->clear();
                pcl::transformPointCloud(*cloud_voxel_tem, *cloud1,
                                         mvGlobalPointCloudsPose[i].inverse().matrix());
                *globalMap += *cloud1;
            }
            is_loop_for_remap = false;
        } else {
            //如果有新点云加入则拼接点云
            PointCloud::Ptr tem_cloud1(new PointCloud());
            std::cout << GREEN << "mvPosePointClouds.size(): " << mvGlobalPointCloudsPose.size() << std::endl;
            tem_cloud1 = generatePointCloud(colorImgs.back(), depthImgs.back(),
                                            mvGlobalPointCloudsPose.back());

            if (tem_cloud1->empty())
                continue;
            *globalMap += *tem_cloud1;
            sensor_msgs::PointCloud2 local;
            pcl::toROSMsg(*tem_cloud1, local);
            local.header.stamp = ros::Time::now();
            local.header.frame_id = "local";
            pub_local_pointcloud.publish(local);
        }

        lastKeyframeSize = mvGlobalPointCloudsPose.size();
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*globalMap, output);
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "world";
        pub_global_pointcloud.publish(output);
        pcl_viewer.showCloud(globalMap);
    //                std::cout << WHITE << "show global map, size=" << globalMap->points.size() << std::endl;
    }
}
```

增加线程锁是为了避免与后面的回环造成数据冲突，每有一个新的关键帧加入时，将关键帧对应的点云和rgb图像以及此时相机的位姿进行转化，转化到世界坐标系下的彩色点云

### void PointCloudMapper::callback(...)

用于接受订阅消息的回调函数，接收到图像和位姿后将`geometry_msgs::PoseStamped::ConstPtr`的位姿转化为`Eigen::Isometry3f`类型，然后调用`insertKeyFrame(...)`将接收到的数据储存起来

当检测到回环是，不再接受某一关键帧的位姿信息，而是接收BA后的相机轨迹，再将每一帧的轨迹存储到`Eigen::Isometry3f`的`vector`中

```c++
if (is_loop) {
    std::unique_lock<std::mutex> lock_loop(loopUpdateMutex);
    std::vector<Eigen::Isometry3f> poses;
    std::vector<cv::Mat> colors, depths;
    for (long i = 0; i < path->poses.size(); i++) {
        for (long j = i; j < kf_ids.size(); j++) {
            if (kf_ids[j] == long(path->poses[i].header.seq)) {
                geometry_msgs::PoseStamped Tcw = path->poses[i];
                Eigen::Affine3f affine;
                Eigen::Vector3f Oe;
                Oe(0) = Tcw.pose.position.x;
                Oe(1) = Tcw.pose.position.y;
                Oe(2) = Tcw.pose.position.z;
                affine.translation() = Oe;
                Eigen::Quaternionf q;
                q.x() = Tcw.pose.orientation.x;
                q.y() = Tcw.pose.orientation.y;
                q.z() = Tcw.pose.orientation.z;
                q.w() = Tcw.pose.orientation.w;
                Eigen::Matrix3f Re(q);
                affine.linear() = Re;
                affine.translation() = Oe;
                Eigen::Isometry3f T = Eigen::Isometry3f(affine.cast<float>().matrix());
                poses.push_back(T);
                colors.push_back(colorImgs[j]);
                depths.push_back(depthImgs[j]);
                break;
            }
        }
    }
    is_loop = false;
    is_loop_for_remap = true;
    if (poses.empty()) return;
    mvGlobalPointCloudsPose.swap(poses);
    colorImgs.swap(colors);
    depthImgs.swap(depths);
} else if (!is_loop_for_remap) {
    kf_ids.push_back(msgRGB->header.seq);
    insertKeyFrame(color, depth, T);
}
```

### void PointCloudMapper::boolCallback()

检测到回环时调用的回调函数

## orb slam3修改部分

由于当时初学时文档没有写的很完善, 部分.h文件中定义的变量没有写出来,完整修改后的orb slam3的代码可以参考[repo](https://github.com/TioeAre/ORB_SLAM3)

### ros_rgbd.cc

```c++
#include...

using namespace std;

class ImageGrabber {
public:
    vector<unsigned long> key_frame_id;
    uint32_t pub_id = 0;
    ros::NodeHandle nh1;
    ros::Publisher pub_rgb, pub_depth, pub_tcw, pub_camerapath, pub_odom, pub_isLoop;
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

tf::Pose trans_pose(Sophus::SE3f se3, bool &if_empty) {
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
    tf::Vector3 V(tWC.at<float>(0) / 25, tWC.at<float>(1) / 25, tWC.at<float>(2) / 25);
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
    tf::Pose tf_pose = trans_pose(se3, if_empty);
    if (!if_empty) {
        std_msgs::Header header;
        header.stamp = msgRGB->header.stamp;
        header.seq = msgRGB->header.seq;
        header.frame_id = "camera";
        sensor_msgs::Image::ConstPtr rgb_msg = msgRGB;
        sensor_msgs::Image::ConstPtr depth_msg = msgD;

        geometry_msgs::PoseStamped tcw_msg;
        tcw_msg.header = header;
        tf::poseTFToMsg(tf_pose, tcw_msg.pose);

        camerapath.header = header;
        std_msgs::Bool isLoop_msg;
        isLoop_msg.data = mpSLAM->is_loop;
        if (mpSLAM->is_loop) {
            vector<geometry_msgs::PoseStamped> after_loop_poses;
            for (long i = 0; i < key_frame_id.size(); i++) {
                for (long j = 0; j < mpSLAM->current_all_KF.size(); j++) {
                    if (key_frame_id[i] == mpSLAM->current_all_KF[j]->mnId) {
                        tf_pose = trans_pose(mpSLAM->current_all_KF[j]->GetPose(), if_empty);
                        geometry_msgs::PoseStamped tcw_msg1;
                        tf::poseTFToMsg(tf_pose, tcw_msg1.pose);
                        tcw_msg1.header = header;
                        tcw_msg1.header.seq = i;
                        after_loop_poses.push_back(tcw_msg1);
                        break;
                    }
                }
            }
            camerapath.poses.swap(after_loop_poses);
        }
        if (mpSLAM->is_loop)
            pub_isLoop.publish(isLoop_msg);
        if (mpSLAM->is_key_frame) {
            key_frame_id.push_back(mpSLAM->current_KF_mnId);
            pub_camerapath.publish(camerapath);
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
    ...
	is_key_frame = false;
    is_key_frame = mpLocalMapper->is_key_frame;
    is_loop = false;
    is_loop = mpTracker->is_loop;
    current_all_KF = mpAtlas->GetAllKeyFrames();
    if (is_key_frame) current_KF_mnId = mpLocalMapper->GetCurrKF()->mnId;
    if (mpTracker->is_loop){
        std::unique_lock<std::mutex> lock_loop(mpTracker->is_loop_mutex);
        mpTracker->is_loop = false;
    }
    if (mpLocalMapper->is_key_frame){
        std::unique_lock<std::mutex> lock_kf(mpLocalMapper->is_keyframe_mutex);
        mpLocalMapper->is_key_frame = false;
    }
}
```

主要增加了以上用于判断是否为关键帧，是否检测到回环的相关变量和判断，并获取此时所有的关键帧

### LocalMapping.cc

```c++
void LocalMapping::Run() {
    if (CheckNewKeyFrames() && !mbBadImu) {
        {
            std::unique_lock<std::mutex> lock_kf(is_keyframe_mutex);
            is_key_frame = true;
        }
        MapPointCulling();
        ...
    }
}
```

添加了判断是否为关键帧，注意要在构建`LocalMapping`中添加，因为BA时使用的是地图点，但好像并不是所有关键帧都在地图点中。如果在`Track()`中添加的话会导致`keyFrame`与优化后`mpAtlas->GetAllKeyFrames()`返回的数目不一致

### LoopClosing.cc

```c++
while(1){
	...
	if (mbLoopDetected) {
        ...
        {
            std::unique_lock<std::mutex> lock_loop(mpTracker->is_loop_mutex);
            mpTracker->is_loop = true;
        }
        mpLoopLastCurrentKF->SetErase();
        ...
    }
}
```

同样添加用于判断是否检测到了回环

## 运行效果

[bilibili](https://www.bilibili.com/video/BV1oa4y1M7Hk/?vd_source=648134b4607a710e0bb6f95aa5fcfd98)

![ijKBfx.png](https://i.328888.xyz/2023/04/04/ijKBfx.png)

使用intel relsense d455回绕一圈后构建的点云图

