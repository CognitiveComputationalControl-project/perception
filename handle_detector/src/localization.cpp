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
  std::string range_sensor_frame;

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
ros::Subscriber sub;

void chatterCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // check whether input frame is equivalent to range sensor frame constant
  std::string input_frame = input->header.frame_id;
  std::cout << input_frame << std::endl;
  if (input_frame.compare(RANGE_SENSOR_FRAME) != 0)
  {
/*    printf("Input frame %s is not equivalent to output frame %s ! Exiting ...\n", input_frame.c_str(),
*//*           RANGE_SENSOR_FRAME.c_str());
*/    std::exit (EXIT_FAILURE);
  }
/*  printf("input frame: %s\noutput frame: %s\n", input_frame.c_str(), RANGE_SENSOR_FRAME.c_str());
*/
  // convert ROS sensor message to PCL point cloud
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromROSMsg(*input, *cloud);
  g_has_read = true;
/*  ROS_INFO("image read");
*/  // organize point cloud for Organized Nearest Neighbors Search
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
  // search grasp affordances
  double start_time = omp_get_wtime();
  g_cylindrical_shells = g_affordances.searchAffordances(g_cloud, &g_transform);
  if (g_cylindrical_shells.size() == 0)
  {
    printf("No handles found!\n");
    g_prev_time = omp_get_wtime();
    return;
  }
  // search handles
  g_handles = g_affordances.searchHandles(g_cloud, g_cylindrical_shells);
}

bool localize(handle_detector::localize_handle::Request  &req, handle_detector::localize_handle::Response &res)
{
  ROS_INFO("localization service launched");
  Visualizer visualizer(g_update_interval);
  std::vector<visualization_msgs::MarkerArray> marker_arrays;
  visualization_msgs::MarkerArray marker_array_msg_handles;

  if (g_has_read)
  {
/*    ROS_INFO("update visualization");
*/    visualizer.createHandles(g_handles, range_sensor_frame, marker_arrays, marker_array_msg_handles);

    g_has_read = false;
  }
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
  // set point cloud source from launch file
  int point_cloud_source;
  node.param("point_cloud_source", point_cloud_source, SENSOR);
/*  printf("point cloud source: %i\n", point_cloud_source);
*/  // set point cloud update interval from launch file
  node.param("update_interval", g_update_interval, 10.0);
  // read parameters
  g_affordances.initParams(node);

  service = node.advertiseService("localize_handle", localize);  

  // wait for and then lookup transform between camera frame and base frame
  tf::TransformListener transform_listener;
  while(ros::ok()){
    // create subscriber for camera topic
/*    printf("Reading point cloud data from sensor topic: %s\n", RANGE_SENSOR_TOPIC.c_str());
*/  range_sensor_frame = RANGE_SENSOR_FRAME;
    sub = node.subscribe(RANGE_SENSOR_TOPIC, 10, chatterCallback);
    try{ 
    transform_listener.waitForTransform(BASE_FRAME, RANGE_SENSOR_FRAME,ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform(BASE_FRAME, RANGE_SENSOR_FRAME, ros::Time(0), g_transform);
    break;
    }   
    catch (tf::TransformException &ex){
    }
  }

                                                                                            
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
