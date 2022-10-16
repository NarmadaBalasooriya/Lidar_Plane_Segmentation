# Lidar Plane Segmentation

This repo contains the ROS package for the Lidar Plane Segmentation using Octree and SACSegmentation in PCL C++. 

To run the code, clone the repo into your catkin worksapce src folder and follow the below commands.

```bash
$ cd catkin_ws/src
$ chmod +x run.sh
$ ./run.sh
```

The package subscribe to rostopics to retrieve the point cloud map based on the current position given by the poses array. To change the topic, change the followings.

In line no.66 and 67 of mapSegmentation.cpp file, change the respective topics.

```c++
// subscriber node for the point cloud map topic
cloudMap = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_map", 1, &mapSegmentation::segmentHandler, this,ros::TransportHints().tcpNoDelay());

// subscriber node for the poses topic
posePath = nh.subscribe<nav_msgs::Path>("/aft_mapped_path", 1, &mapSegmentation::pathHandler, this, ros::TransportHints().tcpNoDelay());

```
To publish the segmented plane, change the topic in the line 63 and the frame id in line 151.

```c++
// publisher node for the segmented and colored planes
segMap = nh.advertise<sensor_msgs::PointCloud2>("/segmented/map",1);

// publishing to the "/odom" framde id
publishCloud(&segMap, transformedColoredCloud, timeLaserInfoStamp, "/odom");
```
