#pragma once
#include "utility.h"
#include <jsk_rviz_plugins/OverlayText.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

class visualizer : public ParamServer
{

public:
    visualizer();
    
    void run();
    void runVisualizerOri();
    void runVisualizerChg(gtsam::NonlinearFactorGraph::shared_ptr &graph_, gtsam::Values &result);
    void runVisualText();
    
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

    const int getIter()
    {
        return iter_;
    }

    void setMaxIter(int &maxIter)
    {
        maxIter_ = maxIter;
    }

    const int getMaxIter()
    {
        return maxIter_;
    }

    void setCheck(bool &check)
    {
        check_ = check;
    }

    bool getCheck()
    {
        return check_;
    }

    // void setGraph(gtsam::NonlinearFactorGraph &graph)
    // {
    //     graph_ = graph;
    // }

    // void setResult(gtsam::Values &result)
    // {
    //     result_ = result;
    // }


private:
    ros::Publisher oriConstraintPub_;
    ros::Publisher chgConstraintPub_;
    ros::Publisher textPub_;

    ros::Publisher posePub_;
    std::shared_ptr<std::thread> publishText_;
    std::vector<vertexInfo> vecVertexInfo_;
    std::vector<edgeInfo> vecEdgeInfo_;

    int iter_, maxIter_;
    bool check_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values result_;

    
};
