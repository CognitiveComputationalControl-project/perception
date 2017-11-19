#include "ros/ros.h"
#include "object_scan.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>



using namespace Eigen;



int num_x=12;
int num_y=12;
ros::Rate loop_rate(10);

//variables & functions for service
bool g_caught_sigint=false;

void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Human_scanner");

  Handle_manager object_tracker;
  
  // ros::Subscriber Object_detected_sub;  
  ros::Subscriber Detected_handle_sub;
  ros::Subscriber Marker_array_sub;
  ros::Subscriber sub;
  ros::NodeHandle n;
  object_tracker.handletarget_pub=n.advertise<visualization_msgs::Marker>("/detected_final_handle_marker",50,true);
  object_tracker.grasp_pub = n.advertise<geometry_msgs::PoseStamped> ("handle_detector/grasp_point", 10,true);

  object_tracker.sub = n.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 10, cloud_callback);
  object_tracker.client = n.serviceClient<handle_detector::localize_handle>("localization/localize_handle");
  object_tracker.service = node.advertiseService("track_handle", track_handle);  

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();  
  }
  return 0;
}




