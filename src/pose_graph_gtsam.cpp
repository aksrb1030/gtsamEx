#include "pose_graph_gtsam.h"

poseGraph::poseGraph(/* args */)
{
    AllocateMemory();
}

void poseGraph::AllocateMemory()
{
    // initial.reset(new gtsam::Values);
    // graph.reset(new gtsam::NonlinearFactorGraph);
}

void poseGraph::ConfigCallBack(gtsam_ex::gtsamConfig &config, uint32_t level)
{
    configIter_ = config.iterlation;
    configStart_ = config.startBool;

    std::cout << "configStart_ : " << configStart_ << "\n";

    v_.setIter(configIter_);
    v_.setCheck(configStart_);
    v_.pubText();
}

void poseGraph::readG2o()
{
    int id = 0;
    std::ifstream fin(g2oPathTopic_);

    int num = 0;

    std::vector<vertexInfo> vecVertexInfo;
    std::vector<edgeInfo> vecEdgeInfo;

    if (fin.is_open())
    {
        while (!fin.eof())
        {
            std::string type;
            fin >> type;
            
            if (type == "VERTEX_SE3:QUAT")
            {
                vertexInfo vertex;
                fin >> vertex.id >> vertex.x >> vertex.y >> vertex.z >> vertex.qx >> vertex.qy >> vertex.qz >> vertex.qw;
                vecVertexInfo.push_back(vertex);
            }
            else if (type == "EDGE_SE3:QUAT")
            {
                edgeInfo edge;
                fin >> edge.id_1 >> edge.id_2;
                fin >> edge.x >> edge.y >> edge.z;
                fin >> edge.qx >> edge.qy >> edge.qz >> edge.qw ;

                
                for (int i = 0; i < 6; ++i)
                {
                    for (int j = i; j < 6; ++j)
                    {
                        double mValue;
                        fin >> mValue;
                        edge.infoM6x6(i, j) = edge.infoM6x6(j, i) = mValue;
                    }
                }

                vecEdgeInfo.push_back(edge);
            }
        }
    }
    else
    {
        std::cout << "Don't find g2o file! Please check file path in data/param.yaml" << "\n";
        exit(1);
    }
    
     if (vecVertexInfo.size() != 0 || vecEdgeInfo.size() != 0)
    {
        optimThread_ = std::make_shared<std::thread>(&poseGraph::Optimization, this, std::ref(vecVertexInfo), std::ref(vecEdgeInfo));
        v_.setVertexVec(vecVertexInfo);
        v_.setEdgeVec(vecEdgeInfo);
        v_.runVisualizerOri();
    }
}

// Opitmizaion 코드도 thread로 구성해야함
void poseGraph::Optimization(std::vector<vertexInfo> &vecVertex, std::vector<edgeInfo> &vecEdge)
{
    ros::Rate r(0.5);
    while (ros::ok())
    {
        if(v_.getVetexVec().size() == 0 && v_.getEdgeVec().size() == 0)
        {
            continue;
        }

        if (configStart_ == true)
        {

            gtsam::NonlinearFactorGraph::shared_ptr graph(new gtsam::NonlinearFactorGraph);
            gtsam::Values::shared_ptr initial(new gtsam::Values);

            int vertexCnt = 0, edgeCnt = 0;

            for (auto data : v_.getVetexVec())
            {
                gtsam::Rot3 R = gtsam::Rot3::Quaternion(data.qw, data.qx, data.qy, data.qz);
                gtsam::Point3 t(data.x, data.y, data.z);
                initial->insert(data.id, gtsam::Pose3(R, t));
                vertexCnt++;
            }

            for (auto data : v_.getEdgeVec())
            {   
                gtsam::Key vertexI, vertexJ;

                gtsam::Rot3 R = gtsam::Rot3::Quaternion(data.qw, data.qx, data.qy, data.qz);
                gtsam::Point3 t(data.x, data.y, data.z);

                gtsam::Matrix infoMat = gtsam::I_6x6;
                infoMat.block<3, 3>(0, 0) = data.infoM6x6.block<3, 3>(3, 3); //  cov rotation
                infoMat.block<3, 3>(3, 3) = data.infoM6x6.block<3, 3>(0, 0); //  cov translation
                infoMat.block<3, 3>(0, 3) = data.infoM6x6.block<3, 3>(0, 3); //  off diagonal
                infoMat.block<3, 3>(3, 0) = data.infoM6x6.block<3, 3>(3, 0); //  off diagonal

                // gaussian noise model
                gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(infoMat);

                // add edge factor
                gtsam::NonlinearFactor::shared_ptr factor(
                    new gtsam::BetweenFactor<gtsam::Pose3>(data.id_1, data.id_2, gtsam::Pose3(R, t), model));

                graph->add(factor);
                edgeCnt++;
            }

            // 첫 번째 정점을 고정, gtsma에서 prior factor를 추가

            // 고정하는 이유 : 불확실성을 줄이기 위해.
            // Prior factor는 노드에 대한 사전 정보를 제공하고 graph 추정 문제를 해결하는데 도움
            gtsam::NonlinearFactorGraph graphWithPrior = *graph;
            gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());

            // 첫번째 fac otr에만 prior 추가
            gtsam::Key fisrtKey = 0;
            for (const gtsam::Values::ConstKeyValuePair &keyValue : *initial)
            {
                graphWithPrior.add(gtsam::PriorFactor<gtsam::Pose3>(
                    keyValue.key, keyValue.value.cast<gtsam::Pose3>(), priorModel));
                break;
            }
            ROS_INFO("\033[1;32m----> Prepare Optimization.\033[0m");
            gtsam::LevenbergMarquardtParams paramLM;
            paramLM.setVerbosity("ERROR");
            paramLM.setMaxIterations(configIter_);
            paramLM.setLinearSolverType("MULTIFRONTAL_QR");
            gtsam::LevenbergMarquardtOptimizer optimizer(graphWithPrior, *initial, paramLM);
            gtsam::Values result = optimizer.optimize();

            std::cout << "====== Optimization complete ======" << std::endl;
            std::cout << "Iterlation : " << configIter_ << "\n";
            std::cout << "initial error: " << graph->error(*initial) << std::endl;
            std::cout << "final error: " << graph->error(result) << std::endl;
            std::cout << "==================================="
                      << "\n";

            v_.runVisualizerChg(graph, result);
        }
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gtsam_ex_poseGraph");

    poseGraph pg;

    dynamic_reconfigure::Server<gtsam_ex::gtsamConfig> server;
    dynamic_reconfigure::Server<gtsam_ex::gtsamConfig>::CallbackType f;
    f = boost::bind(&poseGraph::ConfigCallBack, &pg, _1, _2);
    server.setCallback(f);

    pg.readG2o();

    ros::spin();

    return 1;
}