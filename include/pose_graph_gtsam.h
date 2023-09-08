
#include "utility.h"
#include "visualizer.h"
#include "gtsam_ex/gtsamConfig.h"

#include <dynamic_reconfigure/server.h>


// #include <sophus se3.hpp="">
// #include <sophus so3.hpp="">

class poseGraph : public ParamServer
{

private:
    void AllocateMemory();

public:
    poseGraph(/* args */);

    void readG2o();
    void ConfigCallBack(gtsam_ex::gtsamConfig &config, uint32_t level);
    void Optimization(std::vector<vertexInfo> &vecVertex, std::vector<edgeInfo> &vecEdge);
    void SaveOptiResult(gtsam::NonlinearFactorGraph::shared_ptr &graph, gtsam::Values &result);

public : 
    visualizer v_;

    std::vector<vertexInfo> vecVertexInfo_;
    std::vector<edgeInfo> vecEdgeInfo_;

private:
    int configIter_ = 0;
    bool configStart_ = false;

    std::shared_ptr<std::thread> optimThread_;

    // gtsam::NonlinearFactorGraph::shared_ptr graph_;
    // gtsam::Values::shared_ptr initial_;

};

