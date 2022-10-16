#include "utility.h"


class mapSegmentation : public ParamServer
{
    public:
        // Subscriber and publisher variables
        ros::Publisher segMap;
        //ros::Publisher newMap;

        ros::Subscriber cloudMap;
        ros::Subscriber posePath;

        // keypose variable
        pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;

        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoses;
        pcl::PointXYZI cloudPoses;

        pcl::PointCloud<PointType>::Ptr lidarCloudMap;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingPoints;

        ros::Time timeLaserInfoStamp;

        nav_msgs::Path globalPath;

        //color planar segmentation
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidarCloudXYZ;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;

        Eigen::MatrixXf cloudMatrix;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;

        //Octree
        //pcl::octree::OctreePointCloudPointVector<PointType>::Ptr OctreeT;
        float resolution_octree = 10.0f;
        float slope_thresh = 0.1;
        
        //current pose
        int currentPose;

        // Write poses to a file
        //ofstream posesFile;

        //color values -> original
        uint8_t r = 255;
        uint8_t g = 0;
        uint8_t b = 0;
        int32_t rgb_or = (r << 16) | (g << 8) | b;

        uint8_t r2 = 0;
        uint8_t g2 = 255;
        int32_t rgb_nw = (r2 << 16) | (g2 << 8) | b;


        mapSegmentation()
        {
            segMap = nh.advertise<sensor_msgs::PointCloud2>("/segmented/map",1);
            //newMap = nh.advertise<pcl::PointCloud<PointType>>("/seg/new",1);

            cloudMap = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_map", 1, &mapSegmentation::segmentHandler, this,ros::TransportHints().tcpNoDelay());
            posePath = nh.subscribe<nav_msgs::Path>("/aft_mapped_path", 1, &mapSegmentation::pathHandler, this, ros::TransportHints().tcpNoDelay());

            currentPose = 0;

            //posesFile.open("/home/themi/mapSeg_ws/src/map_segmentation/src/posesList.txt");

            allocateMemory();

        }

        void allocateMemory()
        {
            cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());

            //cloudPoses.reset(new pcl::PointXYZ());

            lidarCloudMap.reset(new pcl::PointCloud<PointType>());
            laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());

            kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
            kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

            kdtreeSurroundingPoints.reset(new pcl::KdTreeFLANN<PointType>());

            cloudRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

            //OctreeT.reset(new pcl::octree::OctreePointCloudPointVector<PointType>());
            

        }

        void segmentHandler(const sensor_msgs::PointCloud2ConstPtr& cloudmsg)
        {
            timeLaserInfoStamp = cloudmsg->header.stamp;
            double tt = cloudmsg->header.stamp.toSec();
            // ROS_INFO("x: %f",tt);
            pcl::fromROSMsg(*cloudmsg, *lidarCloudMap);
            
        }

        void pathHandler(const nav_msgs::PathConstPtr& cloudpath)
        {
            // ROS_INFO("current Pose: %d", currentPose);
            // if(currentPose == 0){
            //     cloudPoses.x = cloudpath->poses.back().pose.position.x;
            //     cloudPoses.y = cloudpath->poses.back().pose.position.y;
            //     cloudPoses.z = cloudpath->poses.back().pose.position.z;
            // }else{
            //     cloudPoses.x = cloudpath->poses[currentPose].pose.position.x;
            //     cloudPoses.y = cloudpath->poses[currentPose].pose.position.y;
            //     cloudPoses.z = cloudpath->poses[currentPose].pose.position.z;
            // }

            
            cloudPoses.x = cloudpath->poses.back().pose.position.x;
            cloudPoses.y = cloudpath->poses.back().pose.position.y;
            cloudPoses.z = cloudpath->poses.back().pose.position.z;

            extractNearPoints();
        }

        void extractNearPoints()
        {
            pcl::PointCloud<PointType>::Ptr surroundingPoints(new pcl::PointCloud<PointType>());
            //pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>());

            //new colored cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedColoredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

            std::vector<int> pointsearchInd;
            std::vector<float> pointSearchSqDist;

            if(lidarCloudMap->size()>0)
            {
                kdtreeSurroundingPoints->setInputCloud(lidarCloudMap);
                kdtreeSurroundingPoints->radiusSearch(cloudPoses, (double)surroundingkeyframeSearchRadius, pointsearchInd, pointSearchSqDist);

                for (int i=0; i < (int)pointsearchInd.size(); ++i)
                {
                    int id = pointsearchInd[i];
                    surroundingPoints->push_back((*lidarCloudMap)[id]);
                }
                //transformedColoredCloud = planarSegmentCloud(surroundingPoints);
                transformedColoredCloud = classicalSegmentCloud(surroundingPoints);
                publishCloud(&segMap, transformedColoredCloud, timeLaserInfoStamp, "/odom");

                currentPose = currentPose + 5;
            }    
       
        }


        pcl::PointCloud<PointType>::Ptr transformCloud(pcl::PointCloud<PointType>::Ptr cloudIn){
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

            float theta = M_PI/2;
            Eigen::Affine3f transform_t = Eigen::Affine3f::Identity();

            transform_t.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

            pcl::transformPointCloud(*cloudIn, *cloudOut, transform_t);

            return cloudOut;
        }


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr classicalSegmentCloud(pcl::PointCloud<PointType>::Ptr cloudIn){

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInterim(new pcl::PointCloud<pcl::PointXYZRGB>());

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());

            cloudRGB->points.resize(cloudIn->size());

            //ROS_INFO("cloudIn size %ld", cloudIn->size());

            for (size_t i=0; i<cloudIn->points.size(); i++){
                cloudRGB->points[i].x = cloudIn->points[i].x;
                cloudRGB->points[i].y = cloudIn->points[i].y;
                cloudRGB->points[i].z = cloudIn->points[i].z;

                cloudRGB->points[i].rgb = rgb_or;
            }

            pcl::octree::OctreePointCloudSearch<PointType> octreeT (resolution_octree);
            
            octreeT.setInputCloud(cloudIn);
            octreeT.addPointsFromInputCloud();

            
            pcl::PointCloud<PointType>::VectorType voxelCenters;
            octreeT.getOccupiedVoxelCenters(voxelCenters);

            for (size_t r=0; r<voxelCenters.size(); ++r){
                Eigen::Vector3f centerVal= voxelCenters[r].getVector3fMap();
                
                std::vector<int> pointIdxVec;

                if(octreeT.voxelSearch(voxelCenters[r],pointIdxVec)){
                    //ROS_INFO("pointIdVec size %ld", pointIdxVec.size());

                    if(pointIdxVec.size()>15){

                        cloudInterim->resize(pointIdxVec.size());

                        TupleList tl;

                        for(size_t i=0;i<pointIdxVec.size(); ++i){


                            cloudInterim->points[i].x = cloudRGB->points[pointIdxVec[i]].x;
                            cloudInterim->points[i].y = cloudRGB->points[pointIdxVec[i]].y;
                            cloudInterim->points[i].z = cloudRGB->points[pointIdxVec[i]].z;
                            cloudInterim->points[i].rgb = rgb_or;

                            tl.push_back(tuple<size_t,size_t>(i,pointIdxVec[i]));

                        }

                        bool isFlat = classicalSegmentFunction(cloudInterim);

                        if(isFlat){
                            for (size_t j=0;j<pointIdxVec.size(); ++j){
                                //ROS_INFO("idx %d", idx);
                                cloudRGB->points[pointIdxVec[j]].r = 0;
                                cloudRGB->points[pointIdxVec[j]].g = 255;
                                cloudRGB->points[pointIdxVec[j]].b = 121;
                            }
                        }else{
                            ROS_INFO("Could not estimate a planar model for the given data\n");                            
                        }
                        
                    }
                                       
                }
            }

            return cloudRGB;
        }


        bool classicalSegmentFunction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn){

            // SACSegmentation
            pcl::ModelCoefficients::Ptr coefficents(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            //create segmentation object
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;

            // set the parameters
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.0001);

            // input cloud and segment
            seg.setInputCloud(cloudIn);
            seg.segment(*inliers, *coefficents);

            float inlierSize = float(inliers->indices.size());
            float cloudSize = float(cloudIn->size());

            //ROS_INFO("test function %ld", inliers->indices.size());
            if(inliers->indices.size() == 0){
                //PCL_ERROR("Could not estimate a planar model for the given data\n");
                ROS_INFO("Error");
                return 0;
            }else{

                float inlier_percentage = ((inlierSize/cloudSize) * 100);
                //ROS_INFO("inlier per %f, %f,  %f", inlierSize, cloudSize, inlier_percentage);
                
                float coeffA = coefficents->values[0];
                float coeffB = coefficents->values[1];

                //ROS_INFO("inlier per %f, %f,  %f", coeffA, coeffB, inlier_percentage);

                //posesFile << inlier_percentage <<" A: " << coeffA << " , B: " << coeffB << "\n";

                if(inlier_percentage > 80.0){

                    if((! isnan(coeffA)) && (! isnan(coeffB)) && (coeffA > 0.9) && (coeffB > -0.9)){
                        return true;
                    }else{
                        return false;
                    }
                }else{
                    return true;
                }              
            }
        }


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr planarSegmentCloud(pcl::PointCloud<PointType>::Ptr cloudIn){
        
            cloudRGB->points.resize(cloudIn->size());
            for (size_t i=0; i<cloudIn->points.size(); i++){
                cloudRGB->points[i].x = cloudIn->points[i].x;
                cloudRGB->points[i].y = cloudIn->points[i].y;
                cloudRGB->points[i].z = cloudIn->points[i].z;

                cloudRGB->points[i].rgb = rgb_or;
            }
            
            pcl::ModelCoefficients::Ptr coefficents(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            //create segmentation object
            pcl::SACSegmentation<PointType> seg;

            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.1);

            seg.setInputCloud(cloudIn);
            seg.segment(*inliers, *coefficents);

            if(inliers->indices.size() == 0){
                PCL_ERROR("Could not estimate a planar model for the given data\n");
                ROS_INFO("Error");
                return cloudRGB;
            }

            for (size_t j = 0; j < inliers->indices.size(); j++){
                //cloudRGB->points[j].rgb = rgb_nw;
                cloudRGB->points[j].r = 0;
                cloudRGB->points[j].g = 255;
                cloudRGB->points[j].b = 123;
            }
            // extract.setInputCloud(cloudIn);
            // extract.setIndices(inliers);
            // extract.setNegative(false);
            // extract.filter(*cloudOut);



            return cloudRGB;

        }

        void extractSurroundingCheck(){
            extractNearPoints();
        }

        // void extractNearCloud(pcl::PointCloud<PointType>::Ptr cloudtoExtract)
        // {

        // }



};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_segmentation");

    mapSegmentation mS;
    mS.extractNearPoints();

    ros::spin();

    return 0;
}