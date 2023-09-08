
#include <visualizer.h>

visualizer::visualizer()
{

    publishText_.reset(new std::thread(std::bind(&visualizer::pubText, this)));

    oriConstraintPub_ = nh_.advertise<visualization_msgs::MarkerArray>("ori_constraint_list", 1, true);
    chgConstraintPub_ = nh_.advertise<visualization_msgs::MarkerArray>("chg_constraint_list", 1, true);
    textPub_ = nh_.advertise<jsk_rviz_plugins::OverlayText>("overlay_text", 1);
}

void visualizer::run()
{
    runVisualizerOri();
}

void visualizer::pubText()
{
    // ros::Rate r(0.5);

    // while (ros::ok())
    // {
        
    //     r.sleep();
    // }
    
    jsk_rviz_plugins::OverlayText text;

    std::string inputText = "Iter : " + std::to_string(iter_);

    text.action = jsk_rviz_plugins::OverlayText::ADD;
    text.width = 100;
    text.height = 20;
    text.left = 10;
    text.top = 10;
    text.text_size = 12;
    text.line_width = 2.0;
    text.font = "DejaVu Sans Mono";
    text.text = "Iter : " + std::to_string(iter_);
    if (check_ == true)
    {
        text.fg_color.r = 25 / 255.0;
        text.fg_color.g = 1.0;
        text.fg_color.b = 240 / 255.0;
        text.fg_color.a = 1.0;
    }
    else
    {
        text.fg_color.r = 255.0 / 255.0;
        text.fg_color.g = 0.0;
        text.fg_color.b = 0.0;
        text.fg_color.a = 1.0;
    }

    text.bg_color.r = 0.0;
    text.bg_color.g = 0.0;
    text.bg_color.b = 0.0;
    text.bg_color.a = 0.8;

    textPub_.publish(text);
}

void visualizer::runVisualizerOri()
{
    int id = 0;

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.6;
    marker.color.g = 0.0;
    marker.color.b = 0.6;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(0);

    visualization_msgs::Marker edge;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.id = 0;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.scale.x = 0.2;
    edge.scale.y = 0.2;
    edge.scale.z = 0.2;
    edge.color.r = 0.0;
    edge.color.g = 1.0;
    edge.color.b = 0.0;
    edge.color.a = 0.6;

    edge.pose.orientation.x = 0.0;
    edge.pose.orientation.y = 0.0;
    edge.pose.orientation.z = 0.0;
    edge.pose.orientation.w = 1.0;

    for (int i = 0; i < vecVertexInfo_.size(); i++)
    {
        marker.id = id;
        marker.pose.position.x = vecVertexInfo_[i].x;
        marker.pose.position.y = vecVertexInfo_[i].y;
        marker.pose.position.z = vecVertexInfo_[i].z;
        markerArray.markers.push_back(visualization_msgs::Marker(marker));
        id++;
    }

    for (int i = 0; i < vecEdgeInfo_.size(); i++)
    {
        edge.points.clear();
        geometry_msgs::Point p;
        p.x = vecVertexInfo_[vecEdgeInfo_[i].id_1].x;
        p.y = vecVertexInfo_[vecEdgeInfo_[i].id_1].y;
        p.z = vecVertexInfo_[vecEdgeInfo_[i].id_1].z;
        edge.points.push_back(p);

        p.x = vecVertexInfo_[vecEdgeInfo_[i].id_2].x;
        p.y = vecVertexInfo_[vecEdgeInfo_[i].id_2].y;
        p.z = vecVertexInfo_[vecEdgeInfo_[i].id_2].z;
        edge.points.push_back(p);
        edge.id = id;
        markerArray.markers.push_back(visualization_msgs::Marker(edge));
        id++;
    }

    oriConstraintPub_.publish(markerArray);
}

void visualizer::runVisualizerChg(gtsam::NonlinearFactorGraph::shared_ptr &graph, gtsam::Values &result)
{
    ros::Rate rate(0.5);

    int id = 0;

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);
   
    for (const gtsam::Values::KeyValuePair key_value : result)
    {
        gtsam::Pose3 solved_pose = key_value.value.cast<gtsam::Pose3>();
        auto t = solved_pose.translation();
        auto R = solved_pose.rotation().quaternion();

        marker.id = id;
        marker.pose.position.x = t.x();
        marker.pose.position.y = t.y();
        marker.pose.position.z = t.z();
        markerArray.markers.push_back(visualization_msgs::Marker(marker));
        id++;


        // fout << "VERTEX_SE3:QUAT " << key_value.key
        //      << " " << t.x() << " " << t.y() << " " << t.z() << " "
        //      << R.x() << " " << R.y() << " " << R.z() << " " << R.w() << " " << std::endl;
        // csv_out << t.x() << ", " << t.y() << ", " << t.z() << std::endl;
    }
    chgConstraintPub_.publish(markerArray);



    // while (ros::ok())
    // {      
    //     // chgConstraintPub_.publish();
    //     rate.sleep();
        
    // }
}