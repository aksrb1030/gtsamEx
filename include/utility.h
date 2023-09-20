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


struct trjInfo
{
    double timestamp;
    double x, y, z;
    double roll, pitch, yaw;
};

struct vertexInfo
{
    int id;
    double x, y, z;
    double qx, qy, qz, qw;
};

struct edgeInfo
{
    int id_1, id_2;
    double x, y, z;
    double qx, qy, qz, qw;
    gtsam::Matrix infoM6x6 = gtsam::I_6x6;
};

class ParamServer
{
private:
    /* data */

public:
    ros::NodeHandle nh_;
    std::string lidarTopic_;
    std::string trjPathTopic_;

    void Tokenize(const std::string &str, std::vector<std::string> &tokens, const std::string &delimiters)
    {
        std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
        std::string::size_type pos = str.find_first_of(delimiters, lastPos);

        while (std::string::npos != pos || std::string::npos != lastPos)
        {
            tokens.push_back(str.substr(lastPos, pos - lastPos));
            lastPos = str.find_first_not_of(delimiters, pos);
            pos = str.find_first_of(delimiters, lastPos);
        }
    }
    

    ParamServer()
    {
        nh_.param<std::string>("/gtsam_ex/pointCloudTopic",lidarTopic_, "/pointCloudTopic");
        nh_.param<std::string>("/gtsam_ex/filePath",trjPathTopic_, "/filePath");
    }

};