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


  ros::NodeHandle n;
  object_tracker.handletarget_pub=n.advertise<visualization_msgs::Marker>("/detected_handle_marker",50,true);
  // object_tracker.Gaze_point_pub= n.advertise<geometry_msgs::Point>("/gazed_point_fixing_node/target_point", 50, true);
  // object_tracker.Gaze_activate_pub= n.advertise<std_msgs::Bool>("/gazed_point_fixing_node/activate", 50, true);
  // global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10, &Handle_manager::global_pose_callback,&object_tracker);
  // jointstates_sub =n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &Handle_manager::joint_states_callback,&object_tracker);
  // Human_markerarray_sub = n.subscribe<visualization_msgs::MarkerArray>("/human_boxes", 10, &Handle_manager::Human_MarkerarrayCallback,&object_tracker);
  // keyboard_sub=n.subscribe<keyboard::Key>("/keyboard/keydown",10, &Handle_manager::keyboard_callback,&object_tracker);
  Detected_handle_sub=n.subscribe<geometry_msgs::Pose>("handle_detector/grasp_point",10,&Handle_manager::grasp_point_callback, &object_tracker);

  ros::Rate loop_rate(25);
  
  while (ros::ok())
  {
     ros::spinOnce();
     loop_rate.sleep();  
  }

  ros::spin();

  return 0;
}




