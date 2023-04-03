#include <iostream>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include "PointCloudMapper.h"


int main(int argc, char **argv) {

    std::string cameraParamFile;

    ros::init(argc, argv, "pointcloud_mapping", ros::init_options::AnonymousName);

    if (!ros::ok()) {
        cout << "ros init error..." << endl;
        return 0;
    }
    ros::start();
    Mapping::PointCloudMapper mapper;
    mapper.viewer();
    cout << "ros shutdown ..." << endl;
    ros::waitForShutdown();
    ros::shutdown();

    return 0;
}
