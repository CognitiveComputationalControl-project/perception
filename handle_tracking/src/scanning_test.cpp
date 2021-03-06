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
#include "handle_tracking/objectfinder.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Human_scanner");

  Handle_manager object_tracker;
  
  object_tracker.handletarget_pub = object_tracker.n.advertise<visualization_msgs::Marker>("/detected_final_handle_marker",50,true);

  object_tracker.client = object_tracker.n.serviceClient<handle_detector::localize_handle>("localization/localize_handle");
 
  object_tracker.service = object_tracker.n.advertiseService<handle_tracking::objectfinder::Request, handle_tracking::objectfinder::Response>("track_handle",boost::bind(&Handle_manager::track_handle, &object_tracker, _1,  _2));  
  
  ros::Rate loop(10);
  while(ros::ok())
  {
    ros::spinOnce();
    loop.sleep();  
  }
  return 0;
}




