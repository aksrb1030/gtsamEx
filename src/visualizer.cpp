
#include <visualizer.h>

visualizer::visualizer()
{

    publishThread_.reset(new std::thread(std::bind(&visualizer::runVisualizerChg, this)));
    oriConstraintPub_ = nh_.advertise<visualization_msgs::MarkerArray>("ori_constraint_list", 1, true);
    chgConstraintPub_ = nh_.advertise<visualization_msgs::MarkerArray>("chg_constraint_list", 1, true);
}

void visualizer::run()
{
    runVisualizerOri();
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

void visualizer::runVisualizerChg()
{
    ros::Rate rate(0.5);

    while (ros::ok())
    {      
        if(check_ == true)
        {
            std::cout << "iter_ : " << iter_ << "\n";
        }


        // chgConstraintPub_.publish();
        // rate.sleep();
        
    }
}