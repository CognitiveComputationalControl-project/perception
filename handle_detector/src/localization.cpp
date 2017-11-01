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
void chatterCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (omp_get_wtime() - g_prev_time < g_update_interval)
    return;

  // check whether input frame is equivalent to range sensor frame constant
  std::string input_frame = input->header.frame_id;
  std::cout << input_frame << std::endl;
  if (input_frame.compare(RANGE_SENSOR_FRAME) != 0)
  {
    printf("Input frame %s is not equivalent to output frame %s ! Exiting ...\n", input_frame.c_str(),
           RANGE_SENSOR_FRAME.c_str());
    std::exit (EXIT_FAILURE);
  }
  printf("input frame: %s\noutput frame: %s\n", input_frame.c_str(), RANGE_SENSOR_FRAME.c_str());

  // convert ROS sensor message to PCL point cloud
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromROSMsg(*input, *cloud);
  g_has_read = true;

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

  // store data to file
  //~ pcl::PointCloud<pcl::PointXYZRGB>::Ptr stored_cloud;
  //~ pcl::fromROSMsg(*input, *stored_cloud);
  //~ pcl::io::savePCDFileASCII("/home/andreas/test_pcd.pcd", *stored_cloud);

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

  // measure runtime
  printf("Affordance and handle search done in %.3f sec.\n", omp_get_wtime() - start_time);

  // store current time
  g_prev_time = omp_get_wtime();
}

int main(int argc, char** argv)
{
  // constants
  const int PCD_FILE = 0;
  const int SENSOR = 1;

  // initialize random seed
  srand (time(NULL));

  // initialize ROS
  ros  ::init(argc, argv, "localization");
  ros::NodeHandle node("~");

  // set point cloud source from launch file
  int point_cloud_source;
  node.param("point_cloud_source", point_cloud_source, SENSOR);
  printf("point cloud source: %i\n", point_cloud_source);

  // set point cloud update interval from launch file
  node.param("update_interval", g_update_interval, 10.0);

  // read parameters
  g_affordances.initParams(node);

  std::string range_sensor_frame;
  ros::Subscriber sub;

  // point cloud read from file
  if (point_cloud_source == PCD_FILE)
  {
    range_sensor_frame = "/map";
    std::string file = g_affordances.getPCDFile();

    // load point cloud from PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *g_cloud) == -1)
    {
      std::cerr << "Couldn't read pcd file" << std::endl;
      return (-1);
    }
    printf("Loaded *.pcd-file: %s\n", file.c_str());

    //~ // search grasp affordances using indices
    //~ g_cylindrical_shells = g_affordances.searchAffordances(g_cloud);

    // search grasp affordances using samples
    double start_time = omp_get_wtime();
    std::vector<int> indices = g_affordances.createRandomIndices(g_cloud, g_affordances.getNumSamples());
    g_cylindrical_shells = g_affordances.searchAffordances(g_cloud, indices);

    // search handles
    g_handles = g_affordances.searchHandles(g_cloud, g_cylindrical_shells);

    // set boolean variable so that visualization topics get updated
    g_has_read = true;

    // measure runtime
    printf("Affordance and handle search done in %.3f sec.\n", omp_get_wtime() - start_time);
  }
  // point cloud read from sensor
  else if (point_cloud_source == SENSOR)
  {
    // wait for and then lookup transform between camera frame and base frame
    tf::TransformListener transform_listener;
    transform_listener.waitForTransform(BASE_FRAME, RANGE_SENSOR_FRAME,ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform(BASE_FRAME, RANGE_SENSOR_FRAME, ros::Time(0), g_transform);

    // create subscriber for camera topic
    printf("Reading point cloud data from sensor topic: %s\n", RANGE_SENSOR_TOPIC.c_str());
    range_sensor_frame = RANGE_SENSOR_FRAME;
    sub = node.subscribe(RANGE_SENSOR_TOPIC, 10, chatterCallback);
  }

  // visualization of point cloud, grasp affordances, and handles
  Visualizer visualizer(g_update_interval);
  sensor_msgs::PointCloud2 pc2msg;
  PointCloud::Ptr cloud_vis(new PointCloud);
  ros::Publisher marker_array_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_all_affordances",
                                                                                    10);
  ros::Publisher marker_array_pub_handles = node.advertise<visualization_msgs::MarkerArray>("visualization_all_handles",
                                                                                            10);
  ros::Publisher marker_array_pub_handle_numbers = node.advertise<visualization_msgs::MarkerArray>(
      "visualization_handle_numbers", 10);
  std::vector<visualization_msgs::MarkerArray> marker_arrays;
  visualization_msgs::MarkerArray marker_array_msg;
  visualization_msgs::MarkerArray marker_array_msg_handles;
  visualization_msgs::MarkerArray marker_array_msg_handle_numbers;

  // publication of grasp affordances and handles as ROS topics
  Messages messages;
  ros::Publisher cylinder_pub = node.advertise<handle_detector::CylinderArrayMsg>("cylinder_list", 10);
  ros::Publisher handles_pub = node.advertise<handle_detector::HandleListMsg>("handle_list", 10);
  ros::Publisher pcl_pub = node.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
  std::vector<ros::Publisher> handle_pubs;
  handle_detector::CylinderArrayMsg cylinder_list_msg;
  handle_detector::HandleListMsg handle_list_msg;
/*  typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
  PCLCloud point_handle;*/
/*  ros::Publisher graspub    = node.advertise<geometry_msgs::Pose> ("/handle_detector/grasp_point", 5);//////////////////////////////////////////////////////////////////
*/  ros::Publisher target_pub =node.advertise<visualization_msgs::Marker>("/handle_target", 10);

  // how often things are published
  ros::Rate rate(10);

  double prev_time = omp_get_wtime();

   while (ros::ok())
   {
    if (g_has_read)
    {
      // create visual point cloud
      cloud_vis = g_affordances.workspaceFilter(g_cloud, &g_transform);
      ROS_INFO("update cloud");

      // create cylinder messages for visualization and ROS topic
      marker_array_msg = visualizer.createCylinders(g_cylindrical_shells, range_sensor_frame);
      cylinder_list_msg = messages.createCylinderArray(g_cylindrical_shells, range_sensor_frame);
      ROS_INFO("update visualization");

      // create handle messages for visualization and ROS topic
      handle_list_msg = messages.createHandleList(g_handles, range_sensor_frame);
      visualizer.createHandles(g_handles, range_sensor_frame, marker_arrays, marker_array_msg_handles);
      
      handle_pubs.resize(g_handles.size());
      for (std::size_t i = 0; i < handle_pubs.size(); i++)
        handle_pubs[i] = node.advertise<visualization_msgs::MarkerArray>(
            "visualization_handle_" + boost::lexical_cast < std::string > (i), 10);

      marker_array_msg_handle_numbers = visualizer.createHandleNumbers(g_handles, range_sensor_frame);

      ROS_INFO("update messages");

      g_has_read = false;
    }

    // publish point cloud
    pcl::toROSMsg(*cloud_vis, pc2msg);
    pc2msg.header.stamp = ros::Time::now();
    pc2msg.header.frame_id = range_sensor_frame;
    pcl_pub.publish(pc2msg);

    // publish cylinders for visualization
    marker_array_pub.publish(marker_array_msg);

    // publish handles for visualization
    for (std::size_t i = 0; i < handle_pubs.size(); i++)
      handle_pubs[i].publish(marker_arrays[i]);

    // publish handles for visualization
    marker_array_pub_handles.publish(marker_array_msg_handles);

    // publish handle numbers for visualization
    marker_array_pub_handle_numbers.publish(marker_array_msg_handle_numbers);

    // publish cylinders as ROS topic
    cylinder_pub.publish(cylinder_list_msg);

    // publish handles as ROS topic
    handles_pub.publish(handle_list_msg);

    // ROS_INFO("handle list published");
    //~ ROS_INFO("published %i grasp affordances for grasping", (int) cylinder_list_msg.cylinders.size());
    //~ ROS_INFO("published %i handles for grasping", (int) handle_list_msg.handles.size());
    //~ for(int i=0; i < handle_list_msg.handles.size(); i++)
    //~ std::cout<<" - handle "<<i<<": "<<handle_list_msg.handles[i].cylinders.size()<<std::endl;
    //~ ROS_INFO("published %i cylinders for visualization", (int) marker_array_msg.markers.size());
    //~ ROS_INFO("published %i handles for visualization", (int) handle_pubs.size());
    //~ for(int i=0; i < marker_arrays.size(); i++)
    //~ std::cout<<" - visual handle "<<i<<": "<<marker_arrays[i].markers.size()<<std::endl;

 
    // tf::TransformListener listener;
    // tf::TransformListener listener2;
/*
    for (int i = 0 ; i < marker_array_msg_handles.markers.size(); i++)
    {
       // geometry_msgs::Vector3Stamped gV, tV;
      if ( (x_max > marker_array_msg_handles.markers[i].pose.position.x) && abs(marker_array_msg_handles.markers[i].pose.position.y)< 0.2)
        { 

          std::cout << " HERE IS ONE VALUE YOU SHOULD BE LOOKING AT MATHAFUCKAAAAAAAAAAAAAAAAAAA"<<std::endl;
          x_max = marker_array_msg_handles.markers[i].pose.position.x;
          
          msg.position.x=marker_array_msg_handles.markers[i].pose.position.x;
          msg.position.y=marker_array_msg_handles.markers[i].pose.position.y;
          msg.position.z=marker_array_msg_handles.markers[i].pose.position.z;

          msg_x=msg.position.x;
          msg_y=msg.position.y;
          msg_z=msg.position.z;

          // gV.vector.x = marker_array_msg_handles.markers[i].pose.position.x;
          // gV.vector.y = marker_array_msg_handles.markers[i].pose.position.y;
          // gV.vector.z = marker_array_msg_handles.markers[i].pose.position.z;
 

          // tf::StampedTransform maptransform;
          // listener2.waitForTransform("head_rgbd_sensor_rgb_frame", BASE_FRAME, ros::Time(0), ros::Duration(1.0));
          // gV.header.stamp = ros::Time();
          // gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
          // //transform
          // // tf::StampedTransform transform;
          // // listener.waitForTransform("head_rgbd_sensor_rgb_frame", BASE_FRAME, ros::Time(0), ros::Duration(1.0));
          // // try{ 
          // // listener.lookupTransform("/head_rgbd_sensor_rgb_frame", BASE_FRAME,ros::Time(0), transform);
          // // }
          // // catch (tf::TransformException &ex){
          // // continue;
          // // }
          // // double offset_x = transform.getOrigin().x() ;        
          // // double offset_y = transform.getOrigin().y() ;        
          // // double offset_z = transform.getOrigin().z() ;        
          // double offset_x =0.22;        
          // double offset_y =0.15 ;        
          // double offset_z = 1.06586  ;       
          // listener2.transformVector(std::string("/base_link"), gV, tV);
          // msg.position.x = tV.vector.x +0.0;
          // msg.position.y = tV.vector.y +0.0;
          // msg.position.z = tV.vector.z +0.0;
          graspub.publish(msg);
                   
          /////////// marker for handle
          // marker_handle.header.frame_id = "base_link"; 
          // marker_handle.header.stamp = ros::Time::now();
          // marker_handle.id = 0;
          // uint32_t shape = visualization_msgs::Marker::SPHERE;
          // marker_handle.type = shape;

          // marker_handle.pose.position.x = msg.position.x;
          // marker_handle.pose.position.y = msg.position.y;
          // marker_handle.pose.position.z = msg.position.z;

          // marker_handle.pose.orientation.x = 0.0;
          // marker_handle.pose.orientation.y = 0.0;
          // marker_handle.pose.orientation.z = 0.0;
          // marker_handle.pose.orientation.w = 1.0;

          // double temp_dist=0.5;
          
          // //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
          // marker_handle.scale.x = std::abs(temp_dist);
          // marker_handle.scale.y = std::abs(temp_dist);
          // marker_handle.scale.z = std::abs(temp_dist);

          // marker_handle.color.r = 0.0;
          // marker_handle.color.g = 0.7;
          // marker_handle.color.b = 0.2;
          // marker_handle.color.a = 0.85;

          // target_pub.publish(marker_handle);
        }
    }

    // msg_trans = msg;
    if (msg_trans.position.x == msg.position.x)
    {ite=ite+1;
      if (ite>10)
        {break;}
    }
   
*/

     ros::spinOnce();
     rate.sleep();
   }
  
  std::cout << "switching to publishing pose only"<<std::endl;

  //while (ros::ok())
  //{
      //msg.position.x=msg_x;
      //msg.position.y=msg_y;
      //msg.position.z=msg_z;
      //graspub.publish(msg);
      //ros::spinOnce();
      //rate.sleep();
  //}
  return 0;
}
