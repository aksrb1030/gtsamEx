#pragma once
#include "utility.h"
// #include "pose_graph_gtsam.h"

#include <jsk_rviz_plugins/OverlayText.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

class visualizer : public ParamServer
{

public:
    visualizer();
    
    void run();
    void runVisualizerOri();
    void runVisualizerChg();
    std::vector<vertexInfo> getVetexVec()
    {
        return vecVertexInfo_;
    }

    void setVertexVec(std::vector<vertexInfo> &vecVertexInfo)
    {
        vecVertexInfo_ = vecVertexInfo;
    }

    std::vector<edgeInfo> getEdgeVec()
    {
        return vecEdgeInfo_;
    }

    void setEdgeVec(std::vector<edgeInfo> &vecEdgeInfo)
    {
        vecEdgeInfo_ = vecEdgeInfo;
    }

    void setIter(int &iter)
    {
        iter_ = iter;
    }

    int getIter()
    {
        return iter_;
    }

    void setCheck(bool &check)
    {
        check_ = check;
    }

    bool getCheck()
    {
        return check_;
    }


private:
    ros::Publisher oriConstraintPub_;
    ros::Publisher chgConstraintPub_;

    ros::Publisher posePub_;
    std::shared_ptr<std::thread> publishThread_;
    std::vector<vertexInfo> vecVertexInfo_;
    std::vector<edgeInfo> vecEdgeInfo_;

    int iter_;
    bool check_;
    
};
