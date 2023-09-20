
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

    
    void ConfigCallBack(gtsam_ex::gtsamConfig &config, uint32_t level);
    void Optimization(std::vector<vertexInfo> &vecVertex, std::vector<edgeInfo> &vecEdge);
    void SaveOptiResult(gtsam::NonlinearFactorGraph::shared_ptr &graph, gtsam::Values &result);
    bool MakeG2OFormat();
    bool ReadG2o();
    bool ReadTrj();

public : 
    visualizer v_;

    std::vector<vertexInfo> vecVertexInfo_;
    std::vector<edgeInfo> vecEdgeInfo_;
    std::vector<trjInfo> vecTrjInfo_;

private:

    int checkIter_;
    int configIter_ = 0;
    bool configStart_ = false;

    std::shared_ptr<std::thread> optimThread_;
};

