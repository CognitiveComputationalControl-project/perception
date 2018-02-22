#include "handle_detector/affordances.h"
#include "handle_detector/CylinderArrayMsg.h"
#include "handle_detector/CylinderMsg.h"
#include "handle_detector/HandleListMsg.h"
#include <ctype.h>
#include "handle_detector/cylindrical_shell.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include <iostream>
#include "handle_detector/messages.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <stdlib.h> 
#include <stdio.h>
#include <string.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <vector>
#include "handle_detector/visualizer.h"
#include "geometry_msgs/Pose.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "handle_detector/localize_handle.h"
#define EIGEN_DONT_PARALLELIZE

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const std::string BASE_FRAME = "base_link";
const std::string RANGE_SENSOR_FRAME = "head_rgbd_sensor_rgb_frame";
const std::string RANGE_SENSOR_TOPIC = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";

// input and output ROS topic data
PointCloud::Ptr g_cloud(new PointCloud);
Affordances g_affordances;
std::vector<CylindricalShell> g_cylindrical_shells;
std::vector<std::vector<CylindricalShell> > g_handles;
tf::StampedTransform g_transform;
static geometry_msgs::Pose msg;
geometry_msgs::Pose msg_trans;

visualization_msgs::Marker marker_handle;
double msg_x=0.0;
double msg_y=0.0;
double msg_z=0.0;

// visualization_msgs::MarkerArray marker_handle;

// synchronization
double g_prev_time;
double g_update_interval;
bool g_has_read = false;
double x_max = 10000;
int ite = 0;
ros::ServiceServer service;

bool localize(handle_detector::localize_handle::Request  &req, handle_detector::localize_handle::Response &res)
{
  ROS_INFO("localization service launched");
  tf::TransformListener transform_listener;
  while(ros::ok()){
    try{ 
    transform_listener.waitForTransform(BASE_FRAME, RANGE_SENSOR_FRAME,ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform(BASE_FRAME, RANGE_SENSOR_FRAME, ros::Time(0), g_transform);
    ROS_INFO("transform sensor to base found");
    break;
    }   
    catch (tf::TransformException &ex){
    }
  }    

  ///////////checkit
  Visualizer visualizer(g_update_interval);
  std::vector<visualization_msgs::MarkerArray> marker_arrays;
  visualization_msgs::MarkerArray marker_array_msg_handles;
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromROSMsg(req.pointcloud_data, *cloud);
  // organize point cloud for Organized Nearest Neighbors Search
  g_cloud->width = 640;
  g_cloud->height = 480;
  g_cloud->points.resize(g_cloud->width * g_cloud->height);
  for (int i = 0; i < g_cloud->height; i++)
  {
    for (int j = 0; j < g_cloud->width; j++)
    {
      g_cloud->points[i * g_cloud->width + j] = cloud->points[i * g_cloud->width + j];
    }
  }
  ///////////
  // search grasp affordances
  g_cylindrical_shells = g_affordances.searchAffordances(g_cloud, &g_transform);
  // search handles
  g_handles = g_affordances.searchHandles(g_cloud, g_cylindrical_shells);
  visualizer.createHandles(g_handles, RANGE_SENSOR_FRAME, marker_arrays, marker_array_msg_handles);
  //for (int i =0 ; i< marker_array_msg_handles.markers.size();i++)
//{ 
    //ROS_INFO("%d value for marker on x ",marker_array_msg_handles.markers[i].pose.position.x);
//ROS_INFO("%d value for marker on y ",marker_array_msg_handles.markers[i].pose.position.y);
//ROS_INFO("%d value for marker on z ",marker_array_msg_handles.markers[i].pose.position.z);
//}
  res.handle_marker = marker_array_msg_handles;
  return true;

}

int main(int argc, char** argv)
{
  // constants
  const int SENSOR = 1;
  // initialize random seed
  srand (time(NULL));
  // initialize ROS
  ros  ::init(argc, argv, "localization");
  ros::NodeHandle node("~");

  g_affordances.initParams(node);

  service = node.advertiseService("localize_handle", localize);  

  // wait for and then lookup transform between camera frame and base frame
                                                                                     
  // how often things are published
  ROS_INFO("Ready to localize");
  ros::Rate r(10);
  while (ros::ok())
  {
  ros::spinOnce();
  r.sleep();
  }
  return 0;
}
