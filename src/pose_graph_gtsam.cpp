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
}

void poseGraph::SaveOptiResult(gtsam::NonlinearFactorGraph::shared_ptr &graph, gtsam::Values &result)
{
    std::string savePath = "/home/mgkim/catkin_ws/src/gtsamEx/data/result_gtsam_" + std::to_string(configIter_);
    ofstream fout(savePath + ".g2o");
    ofstream csv_out(savePath + ".csv");

    for (const gtsam::Values::KeyValuePair key_value : result)
    {
        gtsam::Pose3 solved_pose = key_value.value.cast<gtsam::Pose3>();
        auto t = solved_pose.translation();
        auto R = solved_pose.rotation().quaternion();
        fout << "VERTEX_SE3:QUAT " << key_value.key
             << " " << t.x() << " " << t.y() << " " << t.z() << " "
             << R.x() << " " << R.y() << " " << R.z() << " " << R.w() << " " << endl;
        csv_out << t.x() << ", " << t.y() << ", " << t.z() << endl;
    }
    // edges
    for (auto &factor : *graph)
    {
        gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr between_factor =
            dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
        if (between_factor)
        {
            gtsam::SharedNoiseModel model = between_factor->noiseModel();
            gtsam::noiseModel::Gaussian::shared_ptr gausissian_model =
                dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(model);
            if (gausissian_model)
            {
                gtsam::Matrix info_mat = gausissian_model->R().transpose() * gausissian_model->R();
                gtsam::Pose3 solved_pose = between_factor->measured();
                auto t = solved_pose.translation();
                auto R = solved_pose.rotation().toQuaternion();

                fout << "EDGE_SE3:QUAT " << between_factor->key1() << " " << between_factor->key2()
                     << " " << t.x() << " " << t.y() << " " << t.z() << " "
                     << R.x() << " " << R.y() << " " << R.z() << " " << R.w() << " ";

                gtsam::Matrix info_g2o = gtsam::I_6x6;
                info_g2o.block(0, 0, 3, 3) = info_mat.block(3, 3, 3, 3); // cov translation
                info_g2o.block(3, 3, 3, 3) = info_mat.block(0, 0, 3, 3); // cov rotation
                info_g2o.block(0, 3, 3, 3) = info_mat.block(0, 3, 3, 3); // off diagonal
                info_g2o.block(3, 0, 3, 3) = info_mat.block(3, 0, 3, 3); // off diagonal
                for (int i = 0; i < 6; ++i)
                {
                    for (int j = 0; j < 6; ++j)
                    {
                        fout << info_g2o(i, j) << " ";
                    }
                }
                fout << endl;
            }
        }
    }
    fout.close();
    csv_out.close();
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
                fin >> edge.qw >> edge.qx >> edge.qy >> edge.qz;
                fin >> edge.x >> edge.y >> edge.z;

                // gtsam::Matrix infoM6x6 = gtsam::I_6x6;
                
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
    
     if (vecVertexInfo.size() != 0 || vecEdgeInfo.size() != 0)
    {
        // optimThread_->join();
        // optimThread_.reset(new std::thread(std::bind(&visualizer::poseGraph, this)));

        std::shared_ptr<std::thread> optimThread = std::make_shared<std::thread>(&poseGraph::Optimization, this, std::ref(vecVertexInfo), std::ref(vecEdgeInfo));


        v_.setVertexVec(vecVertexInfo);
        v_.setEdgeVec(vecEdgeInfo);
        v_.runVisualizerOri();
    }
}

// Opitmizaion 코드도 thread로 구성해야함
void poseGraph::Optimization(std::vector<vertexInfo> &vecVertex, std::vector<edgeInfo> &vecEdge)
{
    std::cout << "Optimization()" << "\n";
    gtsam::NonlinearFactorGraph::shared_ptr graph(new gtsam::NonlinearFactorGraph);
    std::cout <<"1" << "\n";
    gtsam::Values::shared_ptr initial(new gtsam::Values);
    std::cout <<"2" << "\n";

    for (auto data : vecVertex)
    {
        std::cout << "data.id : " <<data.id << "\n";
        std::cout << "data.x : " << data.x << " data.y : " << data.y << " data.z :" << data.z << "\n";
        std::cout << "================================" << "\n";
    }

    exit(1);

    int vertexCnt = 0, edgeCnt = 0;

    for (auto data : vecVertex)
    {

        std::cout <<"data.id : " <<data.id << " data.x : " << data.x << " " << data.y << " " << data.z << "\n";
        std::cout << "========================" << "\n";
        gtsam::Rot3 R = gtsam::Rot3::Quaternion(data.qw, data.qx, data.qy, data.qz);
        gtsam::Point3 t(data.x, data.y, data.z);
        initial->insert(data.id, gtsam::Pose3(R, t));
        vertexCnt++;
    }

    std::cout << "3"<< "\n";

    for (auto data : vecEdge)
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
    std::cout << "4"<< "\n";

    ros::Rate r(0.1);
    while (ros::ok())
    {
        if (configStart_ == true)
        {
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

            std::cout << "====== Optimization complete ======" << endl;
            std::cout << "Iterlation : " << configIter_ << "\n";
            std::cout << "initial error: " << graph->error(*initial) << endl;
            std::cout << "final error: " << graph->error(result) << endl;
            std::cout << "==================================="
                      << "\n";

            // SaveOptiResult(graph, result);
        }
        else
        {
            std::cout << "Path : " << g2oPathTopic_ << "\n";
            std::cout << "Do not find file"
                      << "\n";
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