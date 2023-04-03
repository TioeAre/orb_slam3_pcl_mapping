#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/compression_profiles.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace Mapping {

    class PointCloudMapper {
    public:
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        bool mbuseExact, mbuseCompressed = false;
        size_t queueSize = 10;
    public:
        PointCloudMapper();
        ~PointCloudMapper();
        void viewer();

    protected:
        bool is_loop = false;
        std::string topicColor, topicDepth, topicTcw, topicIsLoop, topicPath;
        bool mbKeyFrameUpdate = false;  //有新的关键帧插入
        unsigned int index = 0;
        float mresolution = 0.04;   //点云显示精度
        float mcx = 0, mcy = 0, mfx = 0, mfy = 0;
        float mDepthMapFactor = 100; //深度图尺度因子
        size_t lastKeyframeSize = 0;
        size_t mGlobalPointCloudID = 0; //点云ID
        size_t mLastGlobalPointCloudID = 0;
        pcl::VoxelGrid<PointT> voxel; //点云显示精度
        // data to generate point clouds
        cv::Mat depthImg, colorImg, mpose;
        std::vector<cv::Mat> colorImgs, depthImgs;
        std::vector<PointCloud> mvGlobalPointClouds; //关键帧对应的点云序列
        std::vector<Eigen::Isometry3f> mvGlobalPointCloudsPose; //所有关键帧的位姿
        PointCloud::Ptr globalMap, tmp, cloud_voxel_tem, cloud1;

        std::mutex keyframeMutex;
        std::mutex loopUpdateMutex;
        std::mutex shutDownMutex;

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped, nav_msgs::Path> ExactSyncPolicy;

        ros::NodeHandle nh;
        ros::AsyncSpinner spinner;
        ros::Subscriber loop_sub;
        ros::Publisher pub_global_pointcloud, pub_local_pointcloud;
        image_transport::ImageTransport it;
        image_transport::SubscriberFilter *subImageColor, *subImageDepth;
        message_filters::Subscriber<geometry_msgs::PoseStamped> *tcw_sub;
        message_filters::Subscriber<nav_msgs::Path> *path_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *pointcloud_sub;
        message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    protected:
        PointCloud::Ptr generatePointCloud(cv::Mat &color, cv::Mat &depth, Eigen::Isometry3f &T);
        void readParam();
        void getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &outputMap);
        void reset();
        void shutdown();
        void callback(const sensor_msgs::Image::ConstPtr msgRGB,
                      const sensor_msgs::Image::ConstPtr msgD, const geometry_msgs::PoseStamped::ConstPtr tcw,
                      const nav_msgs::Path::ConstPtr path);
        void boolCallback(const std_msgs::Bool::ConstPtr &if_loop);
        void insertKeyFrame(cv::Mat &color, cv::Mat &depth, Eigen::Isometry3f &T);
    };
}
#endif // POINTCLOUDMAPPING_H
