#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "door_recognition/DetectDoor.h"
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher door_plane_points_pub;
ros::ServiceServer door_detect_serv;

bool find_door(door_recognition::DetectDoor::Request &req, door_recognition::DetectDoor::Response &res){
	
	ROS_INFO("Extracting door location from given point cloud");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 cloudf;
    cloud->header.frame_id = req.cloud_input.header.frame_id;
	pcl::fromROSMsg(req.cloud_input, *cloud);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
    pcl::PointXYZ pt;

	for (size_t i = 0; i < inliers->indices.size (); ++i)
	{
		pt.x = cloud->points[inliers->indices[i]].x;
		pt.y = cloud->points[inliers->indices[i]].y;
		pt.z = cloud->points[inliers->indices[i]].z;
		cloud_temp->points.push_back(pt);
	}
	cloud_temp->header.frame_id = cloud->header.frame_id;
	// Return service message
    ROS_INFO("Returning Door Extraction Response");
    pcl::toROSMsg(*cloud_temp,cloudf);
   	cloudf.header.frame_id = cloud_temp->header.frame_id;
    res.door_points = cloudf;
    door_plane_points_pub.publish(cloudf);
return true;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "door_detector_server_node");
    ros::Rate r(20);

	ros::NodeHandle nh;
	ROS_INFO("door_detector_service_node started");

	door_detect_serv = nh.advertiseService("detect_door", find_door);

    // Topics that show information on the internals of this node
    ros::NodeHandle pnh("~");
    door_plane_points_pub = pnh.advertise<sensor_msgs::PointCloud2>("door/points", 1, true);

	ros::spin();
}


