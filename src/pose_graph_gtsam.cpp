#include "pose_graph_gtsam.h"

poseGraph::poseGraph(/* args */)
{
    AllocateMemory();
}

void poseGraph::AllocateMemory()
{
}

void poseGraph::ConfigCallBack(gtsam_ex::gtsamConfig &config, uint32_t level)
{
    configIter_ = config.iterlation;
    v_.setIter(configIter_);

    ROS_INFO("\033[1;32m----> Set dynamic_reconfigure.\033[0m");
}

int poseGraph::ReadTrj()
{
    std::ifstream ifs(trjPathTopic_);

    double timestamp;
    double x, y, z;
    double roll, pitch, heading;

    if (!ifs.is_open())
    {
        std::cerr << "Do not open file. Check your file path in param.yaml" << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(ifs, line))
    {
        while (!line.empty() && std::isspace(line.back()))
        {
            line.pop_back();
        }

        std::vector<std::string> tokens;
        Tokenize(line, tokens, "\t");

        trjInfo trj;
        trj.timestamp = std::stod(tokens[0]);
        trj.x = std::stod(tokens[1]);
        trj.y = std::stod(tokens[2]);
        trj.z = std::stod(tokens[3]);
        trj.roll = std::stod(tokens[4]);
        trj.pitch = std::stod(tokens[5]);
        trj.yaw = std::stod(tokens[6]);
        vecTrjInfo_.push_back(trj);
    }
    ifs.close();
    return 0;
}

int poseGraph::MakeG2OFormat()
{

    std::vector<vertexInfo> vecVertexInfo;
    std::vector<edgeInfo> vecEdgeInfo;

    std::cout << "MakeG2OFormat()" << "\n";
    if (vecTrjInfo_.size() == 0)
    {
        return 1;
    }

    int num = 0;

    for (auto data : vecTrjInfo_)
    {
        vertexInfo vertex;
        vertex.id = num;
        vertex.x = data.x;
        vertex.y = data.y;
        vertex.z = data.z;

        auto [qw, qx, qy, qz] = eulerToQuaternion(data.roll, data.pitch, data.yaw);

        vertex.qw = qw;
        vertex.qx = qx;
        vertex.qy = qy;
        vertex.qz = qz;
        vecVertexInfo.push_back(vertex);

        num ++;
    }

    for (int i = 1; i < vecTrjInfo_.size(); i++)
    {
        edgeInfo edge;

        edge.id_1 = i-1;
        edge.id_2 = i;

        Eigen::Matrix4f beforePose = createTransformationMatrix(vecTrjInfo_[i - 1].roll, vecTrjInfo_[i - 1].pitch, vecTrjInfo_[i - 1].yaw,
                                                                vecTrjInfo_[i - 1].x, vecTrjInfo_[i - 1].y, vecTrjInfo_[i - 1].z);
        Eigen::Matrix4f afterPose = createTransformationMatrix(vecTrjInfo_[i].roll, vecTrjInfo_[i].pitch, vecTrjInfo_[i].yaw,
                                                                vecTrjInfo_[i].x, vecTrjInfo_[i].y, vecTrjInfo_[i].z);

        Eigen::Isometry3d relativePose((beforePose.inverse() * afterPose).cast<double>());

        Eigen::Vector3d translation = relativePose.translation();
        edge.x = translation.x();
        edge.y = translation.y();
        edge.z = translation.z();

        Eigen::Quaterniond quaternion(relativePose.rotation());
        edge.qw = quaternion.w();
        edge.qx = quaternion.x();
        edge.qy = quaternion.y();
        edge.qz = quaternion.z();

        vecEdgeInfo.push_back(edge);
    }

    if (vecVertexInfo.size() != 0 || vecEdgeInfo.size() != 0)
    {
        optimThread_ = std::make_shared<std::thread>(&poseGraph::Optimization, this, std::ref(vecVertexInfo), std::ref(vecEdgeInfo));
        v_.setVertexVec(vecVertexInfo);
        v_.setEdgeVec(vecEdgeInfo);
        v_.runVisualizerOri();
    }
    

    return 0;
}

int poseGraph::ReadG2o()
{
    int id = 0;
    std::ifstream ifs(trjPathTopic_);

    int num = 0;

    std::vector<vertexInfo> vecVertexInfo;
    std::vector<edgeInfo> vecEdgeInfo;

    if (!ifs.is_open())
    {
        std::cerr << "Do not open file. Check your file path in param.yaml" << std::endl;
        return 1;
    }

    while (!ifs.eof())
    {
        std::string type;
        ifs >> type;

        if (type == "VERTEX_SE3:QUAT")
        {
            vertexInfo vertex;
            ifs >> vertex.id >> vertex.x >> vertex.y >> vertex.z >> vertex.qx >> vertex.qy >> vertex.qz >> vertex.qw;
            vecVertexInfo.push_back(vertex);
        }
        else if (type == "EDGE_SE3:QUAT")
        {
            edgeInfo edge;
            ifs >> edge.id_1 >> edge.id_2;
            ifs >> edge.x >> edge.y >> edge.z;
            ifs >> edge.qx >> edge.qy >> edge.qz >> edge.qw;

            for (int i = 0; i < 6; ++i)
            {
                for (int j = i; j < 6; ++j)
                {
                    double mValue;
                    ifs >> mValue;
                    edge.infoM6x6(i, j) = edge.infoM6x6(j, i) = mValue;
                }
            }

            vecEdgeInfo.push_back(edge);
        }
    }

    if (vecVertexInfo.size() != 0 || vecEdgeInfo.size() != 0)
    {
        optimThread_ = std::make_shared<std::thread>(&poseGraph::Optimization, this, std::ref(vecVertexInfo), std::ref(vecEdgeInfo));
        v_.setVertexVec(vecVertexInfo);
        v_.setEdgeVec(vecEdgeInfo);
        v_.runVisualizerOri();
    }
    return 0;
}

// Opitmizaion 코드도 thread로 구성해야함
void poseGraph::Optimization(std::vector<vertexInfo> &vecVertex, std::vector<edgeInfo> &vecEdge)
{
    ros::Rate r(0.5);
    while (ros::ok())
    {
        if (v_.getVetexVec().size() == 0 && v_.getEdgeVec().size() == 0)
            continue;

        if (checkIter_ == configIter_)
            continue;

        // if (configIter_  == 0)
        //     continue;

        std::cout << "configIter_ : " << configIter_ << "\n";

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


            // 공분산의 역행렬  | 불확실성의 역수
            gtsam::Matrix infoMat = gtsam::I_6x6;
            infoMat.block<3, 3>(0, 0) = data.infoM6x6.block<3, 3>(3, 3); //  cov rotation
            infoMat.block<3, 3>(3, 3) = data.infoM6x6.block<3, 3>(0, 0); //  cov translation
            infoMat.block<3, 3>(0, 3) = data.infoM6x6.block<3, 3>(0, 3); //  off diagonal
            infoMat.block<3, 3>(3, 0) = data.infoM6x6.block<3, 3>(3, 0); //  off diagonal

            // std::cout << data.infoM6x6 << "\n\n\n";
            // std::cout << infoMat << "\n";
            // std::cout << "===============================" << "\n";

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
        
        gtsam::LevenbergMarquardtParams paramLM;
        paramLM.setVerbosity("ERROR");
        paramLM.setMaxIterations(configIter_);
        paramLM.setLinearSolverType("MULTIFRONTAL_QR");
        gtsam::LevenbergMarquardtOptimizer optimizer(graphWithPrior, *initial, paramLM);
        gtsam::Values result = optimizer.optimize();

        ROS_INFO("\033[1;32m----> Optimization complete.\033[0m");

        checkIter_ = configIter_;

        int maxIter = optimizer.iterations();

        v_.setMaxIter(maxIter);
        v_.runVisualizerChg(graph, result);
        r.sleep();
    }
}

int main(int argc, char **argv)
{

    // std::this_thread::sleep_for(std::chrono::seconds(10));

    ros::init(argc, argv, "gtsam_ex_poseGraph");

    poseGraph pg;

    dynamic_reconfigure::Server<gtsam_ex::gtsamConfig> server;
    dynamic_reconfigure::Server<gtsam_ex::gtsamConfig>::CallbackType f;
    f = boost::bind(&poseGraph::ConfigCallBack, &pg, _1, _2);
    server.setCallback(f);

    if (pg.trjPathTopic_.substr(pg.trjPathTopic_.size() - 4) == ".txt")
    {
        std::cout << "1" << "\n";
        pg.ReadTrj();
        pg.MakeG2OFormat();
    }
    else
    {
        pg.ReadG2o();
    }

    ros::spin();

    return 1;
}