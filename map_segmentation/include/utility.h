#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/common/common.h>
#include <pcl/common/copy_point.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/octree/octree_search.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <iostream>
#include <fstream>
#include <ctime>
#include <iterator>
#include <string>
#include <limits>
#include <thread>
#include <array>

#include <stdlib.h>

using namespace std;

typedef pcl::PointXYZI PointType;

typedef vector< tuple<size_t, size_t> > TupleList;

class ParamServer{

    public:

        ros::NodeHandle nh;

        //Topics
        string pointCloudTopic;
        string odomTopic;

        //CPU Params
        int numberOfCores;
        
        //Surrounding map
        float surroundingkeyframeAddingDistThreshold;
        float surroundingkeyframeAddingAngleThreshold;
        float surroundingkeyframeSearchRadius;

        ParamServer(){
            nh.param<std::string>("map_segmentation/pointCloudTopic", pointCloudTopic, "sensor_msgs/PointCloud2");
            nh.param<std::string>("map_segmentation/odomTopic", odomTopic, "nav_msgs/Path");

            nh.param<float>("map_segmentation/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
            nh.param<float>("map_segmentation/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
            nh.param<float>("map_segmentation/surroundingkeyframeSearchRadius", surroundingkeyframeSearchRadius, 50.0);

            nh.param<int>("map_segmentation/numberOfCores",numberOfCores, 2);

            usleep(100);
        }

};

template<typename T>
sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, T thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;

    if(thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}


template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif