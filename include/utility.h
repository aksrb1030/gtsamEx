#pragma once

#include <ros/ros.h>

#include <limits>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <sstream>

#include <gtsam/slam/dataset.h>
#include <gtsam_ex/gtsamConfig.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>


struct vertexInfo
{
    int id;
    float x, y, z;
    float qx, qy, qz, qw;
};

struct edgeInfo
{
    int id_1, id_2;
    float x, y, z;
    float qx, qy, qz, qw;
    gtsam::Matrix infoM6x6 = gtsam::I_6x6;
};

class ParamServer
{
private:
    /* data */

public:
    ros::NodeHandle nh_;
    std::string lidarTopic_;
    std::string g2oPathTopic_;

    

    ParamServer()
    {
        nh_.param<std::string>("/gtsam_ex/g2oPath",g2oPathTopic_, "/g2oPath");
        nh_.param<std::string>("/gtsam_ex/pointCloudTopic",lidarTopic_, "/pointCloudTopic");
    }

};