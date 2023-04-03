#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
#include "PointCloudMapper.h"

#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>

#define RED     "\033[31m"  // Red
#define GREEN   "\033[32m"  //Green
#define WHITE   "\033[37m"  //White

namespace Mapping {

    PointCloudMapper::PointCloudMapper() : nh("~"), spinner(0), it(nh) {
        readParam();
        image_transport::TransportHints hints(mbuseCompressed ? "compressed" : "raw");
        subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
        subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
        tcw_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, topicTcw, queueSize);
        path_sub = new message_filters::Subscriber<nav_msgs::Path>(nh, topicPath, queueSize);
        //接受RGB DepTh 位姿数据
        syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor,
                                                                       *subImageDepth, *tcw_sub, *path_sub);
        syncExact->registerCallback(boost::bind(&PointCloudMapper::callback, this, _1, _2, _3, _4));
        loop_sub = nh.subscribe<std_msgs::Bool>(topicIsLoop, queueSize,
                                                boost::bind(&PointCloudMapper::boolCallback, this, _1));
        voxel.setLeafSize(mresolution, mresolution, mresolution);

        globalMap.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
        tmp.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
        cloud_voxel_tem.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
        cloud1.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

        pub_global_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("Global/PointCloudOutput", 1);
        pub_local_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("Local/PointCloudOutput", 10);
    }

    PointCloudMapper::~PointCloudMapper() {
        shutdown();
    }

    /**
     * 加入关键帧
     * @param color 颜色图
     * @param depth 深度图
     * @param T 相机位姿
     */
    void PointCloudMapper::insertKeyFrame(cv::Mat &color, cv::Mat &depth, Eigen::Isometry3f &T) {
        std::unique_lock<std::mutex> lck(keyframeMutex);
        mvGlobalPointCloudsPose.push_back(T);
        colorImgs.push_back(color.clone());
        depthImgs.push_back(depth.clone());

        mGlobalPointCloudID++;
        mbKeyFrameUpdate = true;
        std::cout << GREEN << "receive a keyframe, id = " << mGlobalPointCloudID << WHITE << std::endl;
    }

    /**
     * 生成点云
     * @param color rgb图像
     * @param depth 深度图
     * @param T 相机位姿
     * @return  转化为世界坐标系下的点云图
     */
    pcl::PointCloud<PointCloudMapper::PointT>::Ptr
    PointCloudMapper::generatePointCloud(cv::Mat &color, cv::Mat &depth, Eigen::Isometry3f &T) {

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        tmp->clear();
        for (int m = 0; m < depth.rows; m += 3) {
            for (int n = 0; n < depth.cols; n += 3) {
                float d = depth.ptr<float>(m)[n] / mDepthMapFactor;
                if (d < 0 || d > 6) {
                    continue;
                }
                PointT p;
                p.z = d;
                p.x = (n - mcx) * p.z / mfx;
                p.y = (m - mcy) * p.z / mfy;

                p.r = color.data[m * color.step + n * color.channels()];
                p.g = color.data[m * color.step + n * color.channels() + 1];
                p.b = color.data[m * color.step + n * color.channels() + 2];
                tmp->points.push_back(p);
            }
        }
        cloud_voxel_tem->clear();
        tmp->is_dense = false;
        voxel.setInputCloud(tmp);
        voxel.setLeafSize(mresolution, mresolution, mresolution);
        voxel.filter(*cloud_voxel_tem);
        cloud1->clear();
        pcl::transformPointCloud(*cloud_voxel_tem, *cloud1, T.inverse().matrix());

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

        std::cout << GREEN << "generate point cloud, id = " << mLastGlobalPointCloudID << ", size="
                  << cloud1->points.size() << "\ncost time: " << time_used.count() * 1000 << " ms ." << WHITE
                  << std::endl;
        mLastGlobalPointCloudID++;

        return cloud1;
    }


    /**
     * 主调用函数，生成点云并显示图像
     */
    void PointCloudMapper::viewer() {
        pcl::visualization::CloudViewer pcl_viewer("viewer");
        size_t N = 0;
        bool KFUpdate = false;
        ros::AsyncSpinner spinner(1);
        spinner.start();
        while (ros::ok()) {
            ros::spinOnce();
            KFUpdate = false;
            {
                std::unique_lock<std::mutex> lck(keyframeMutex);
                N = mvGlobalPointCloudsPose.size();
                KFUpdate = mbKeyFrameUpdate;
                mbKeyFrameUpdate = false;
            }
            if (KFUpdate && N > lastKeyframeSize) {
//                if (colorImgs.size() != depthImgs.size() || depthImgs.size() != mvGlobalPointCloudsPose.size() ||
//                    colorImgs.size() != mvGlobalPointCloudsPose.size())
//                    continue;
                std::unique_lock<std::mutex> lock_loop(loopUpdateMutex);

                PointCloud::Ptr tem_cloud1(new PointCloud());
                std::cout << GREEN << "mvPosePointClouds.size(): " << mvGlobalPointCloudsPose.size() << std::endl;
                tem_cloud1 = generatePointCloud(colorImgs.back(), depthImgs[colorImgs.size() - 1],
                                                mvGlobalPointCloudsPose[colorImgs.size() - 1]);

                if (tem_cloud1->empty())
                    continue;
                *globalMap += *tem_cloud1;

                sensor_msgs::PointCloud2 local;
                pcl::toROSMsg(*tem_cloud1, local);
                local.header.stamp = ros::Time::now();
                local.header.frame_id = "local";
                pub_local_pointcloud.publish(local);

                lastKeyframeSize = mvGlobalPointCloudsPose.size();
                sensor_msgs::PointCloud2 output;
                pcl::toROSMsg(*globalMap, output);
                output.header.stamp = ros::Time::now();
                output.header.frame_id = "world";
                pub_global_pointcloud.publish(output);
                pcl_viewer.showCloud(globalMap);
                std::cout << WHITE << "show global map, size=" << globalMap->points.size() << std::endl;
            }
        }
        spinner.stop();
    }

    void PointCloudMapper::callback(const sensor_msgs::Image::ConstPtr msgRGB,
                                    const sensor_msgs::Image::ConstPtr msgD,
                                    const geometry_msgs::PoseStamped::ConstPtr tcw,
                                    const nav_msgs::Path::ConstPtr path) {

        cv::Mat color, depth, depthDisp;
        geometry_msgs::PoseStamped Tcw = *tcw;
        cv_bridge::CvImageConstPtr pCvImage;

        pCvImage = cv_bridge::toCvShare(msgRGB, "rgb8");
        pCvImage->image.copyTo(color);
        pCvImage = cv_bridge::toCvShare(msgD, msgD->encoding); //imageDepth->encoding
        pCvImage->image.copyTo(depth);
        if (color.type() == CV_16U) {
            cv::Mat tmp;
            color.convertTo(tmp, CV_8U, 0.02);
            cv::cvtColor(tmp, color, CV_GRAY2BGR);
        }
        if (mDepthMapFactor != 1 || depth.type() != CV_32F)
            depth.convertTo(depth, CV_32F);
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
        Eigen::Isometry3f T = Eigen::Isometry3f(affine.cast<float>().matrix()); // 获取位姿矩阵
        insertKeyFrame(color, depth, T);
        if (is_loop) {
            std::vector<Eigen::Isometry3f> poses;
            for (const auto &pose: path->poses) {
                geometry_msgs::PoseStamped Tcw = pose;
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
                Eigen::Isometry3f T = Eigen::Isometry3f(affine.cast<float>().matrix()); // 获取位姿矩阵
                poses.push_back(T);
            }
            std::unique_lock<std::mutex> lock_loop(loopUpdateMutex);
            mvGlobalPointCloudsPose.swap(poses);
        }
    }

    /**
     * 检测回环的回调函数
     * @param if_loop
     */
    void PointCloudMapper::boolCallback(const std_msgs::Bool::ConstPtr &if_loop) {
        is_loop = if_loop->data;
        if (is_loop) {
            std::cout << RED << "detect loop!" << std::endl;
            std::unique_lock<std::mutex> lock_loop(loopUpdateMutex);
            globalMap->clear();
            for (int i = 0; i < mvGlobalPointCloudsPose.size(); i += 1) {
                tmp->clear();
                for (int m = 0; m < depthImgs[i].rows; m += 3) {
                    for (int n = 0; n < depthImgs[i].cols; n += 3) {
                        float d = depthImgs[i].ptr<float>(m)[n] / mDepthMapFactor;
                        if (d < 0 || d > 6) {
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
                pcl::transformPointCloud(*cloud_voxel_tem, *cloud1, mvGlobalPointCloudsPose[i].inverse().matrix());
                *globalMap += *cloud1;
            }
        }
    }

    void PointCloudMapper::reset() {
        mvGlobalPointCloudsPose.clear();
        mvGlobalPointClouds.clear();
        mGlobalPointCloudID = 0;
        mLastGlobalPointCloudID = 0;
    }

    void PointCloudMapper::shutdown() {
        std::unique_lock<std::mutex> lck(shutDownMutex);
        std::string save_path = "/home/tioeare/project/pcl_map_ws/src/resultPointCloudFile.pcd";
        pcl::io::savePCDFile(save_path, *globalMap);
        std::cout << "save pcd files to :  " << save_path << std::endl;
    }

    void PointCloudMapper::readParam() {
        float fx_, fy_, cx_, cy_, resolution_, depthfactor_;
        int queueSize_;
        bool mbuseExact_;
        if (ros::param::get("~topicColor", topicColor));
        else
            topicColor = "/RGBD/RGB/Image";
        if (ros::param::get("~topicDepth", topicDepth));
        else
            topicDepth = "/RGBD/Depth/Image";
        if (ros::param::get("~topicTcw", topicTcw));
        else
            topicTcw = "/RGBD/CameraPose";
        topicIsLoop = "/RGBD/isLoop";
        topicPath = "/RGBD/Path";
        nh.param<float>("fx", fx_, 386.69358913);
        nh.param<float>("fy", fy_, 385.60239709);
        nh.param<float>("cx", cx_, 319.18842075);
        nh.param<float>("cy", cy_, 248.14081752);
        nh.param<float>("resolution", resolution_, 0.05);
        nh.param<float>("depthfactor", depthfactor_, 1000.0);
        nh.param<int>("queueSize", queueSize_, 1);
        nh.param<bool>("buseExact", mbuseExact_, true);
        mbuseExact = mbuseExact_;
        queueSize = queueSize_;
        mcx = cx_;
        mcy = cy_;
        mfx = fx_;
        mfy = fy_;
        mresolution = resolution_;
        mDepthMapFactor = depthfactor_;

        std::cout << "fx: " << mfx << std::endl;
        std::cout << "fy: " << mfy << std::endl;
        std::cout << "cx: " << mcx << std::endl;
        std::cout << "cy: " << mcy << std::endl;
        std::cout << "resolution: " << mresolution << std::endl;
        std::cout << "DepthMapFactor: " << mDepthMapFactor << std::endl;
        std::cout << "queueSize: " << queueSize << std::endl;
        std::cout << "mbuseExact: " << mbuseExact << std::endl;
    }
}//end of namespace
