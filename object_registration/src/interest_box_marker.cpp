#include <signal.h>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <interactive_markers/interactive_marker_server.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
/*#include <pcl/pcd_io.h>
*/
#define MARKER_FRAME "map" //"world" // must be at the fixed frame
#define MARKER_NAME "interestBox"

#define DEFAULT_POINTCLOUD_SUB "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"
#define POINTCLOUD_PUB_NAME "/object_registration_interest_box/boxed_points"
#define STORED_CLOUD_PUB_NAME "/object_registration_interest_box/stored_points"

#define DEFAULT_PC_NAME "boxed_cloud.pcd"

#define X_SIZE_ARROW_1 "arrow_x"
#define Y_SIZE_ARROW_1 "arrow_y"
#define Z_SIZE_ARROW_1 "arrow_z"


typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;

//true if Ctrl-C is pressed
bool g_caught_sigint = false;
boost::mutex cloud_mutex;
ros::Publisher  pointcloud_pub;
ros::Publisher  storedcloud_pub;
tf::TransformListener *tf_listener;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
visualization_msgs::Marker interest_box_marker;

PointCloudRGB::Ptr cloud(new PointCloudRGB);
PointCloudRGB::Ptr boxed_cloud (new PointCloudRGB);
std::string stored_cloud_topic = STORED_CLOUD_PUB_NAME;

// what happens when Ctrl-c is pressed
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


void createBoxMarker(visualization_msgs::InteractiveMarker& int_marker,
                     visualization_msgs::Marker& box_marker_copy){
  int_marker.header.frame_id = MARKER_FRAME;
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = MARKER_NAME;
  int_marker.description = "draggable_box";

  // create a sphere marker to represent the center
  visualization_msgs::Marker sphere_marker;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.scale.x = 0.05;  sphere_marker.scale.y = 0.05;  sphere_marker.scale.z = 0.05;
  sphere_marker.color.r = 0.5;   sphere_marker.color.g = 0.5;   sphere_marker.color.b = 0.5;   sphere_marker.color.a = 0.5;

  //create a box marker to represent the bounding box
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 1.0;  box_marker.scale.y = 1.0; box_marker.scale.z = 1.0;
  box_marker.color.r = 0.0;  box_marker.color.g = 0.0; box_marker.color.b = 1.0; box_marker.color.a = 0.5;

  // Copy this marker for initialization purposes
  box_marker_copy = box_marker;

  // create a controller to translate the box in 3D by dragging it
  visualization_msgs::InteractiveMarkerControl box_translate_control;
  box_translate_control.name = "click_and_move_3d";
  box_translate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  box_translate_control.always_visible = true;

  // Use these markers to represent the clickable objects
  box_translate_control.markers.push_back(sphere_marker);
  box_translate_control.markers.push_back(box_marker);

  // add the control to the interactive marker
  int_marker.controls.push_back( box_translate_control );
}

void createArrowMarker(const std::string arrow_name, const float box_size_in_dimension,
                       visualization_msgs::InteractiveMarker& int_marker_arrow){
  // Initialize the Markers we need
  visualization_msgs::Marker arrow_marker;
  visualization_msgs::InteractiveMarkerControl size_arrow_control;

  // Initialize the Pose of the Arrow
  int_marker_arrow.header.frame_id = MARKER_FRAME;
  int_marker_arrow.header.stamp = ros::Time::now();

  // Initialize the visualization of the arrow
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.scale.x = 0.25; arrow_marker.scale.y = 0.1;  arrow_marker.scale.z = 0.1;
  arrow_marker.color.r = 0.0;  arrow_marker.color.g = 1.0;  arrow_marker.color.b = 0.0; arrow_marker.color.a = 0.5;

  // Set the pose, orientation and names
  if (arrow_name.compare(X_SIZE_ARROW_1) == 0){
      // Set initial interactive marker pose
      int_marker_arrow.pose.position.x = box_size_in_dimension/2.0;       
      // Set Arrow Orientation      
      arrow_marker.pose.orientation.x = 0.0;   
      arrow_marker.pose.orientation.y = 0.0;   
      arrow_marker.pose.orientation.z = 0.0;  
      arrow_marker.pose.orientation.w = 1.0;

      int_marker_arrow.name = X_SIZE_ARROW_1;
      size_arrow_control.name = "move_x";      
  }else if ( arrow_name.compare(Y_SIZE_ARROW_1) == 0) {
      // Set initial interactive marker pose    
      int_marker_arrow.pose.position.y = box_size_in_dimension/2.0;
      
      // Set Arrow Orientation
      arrow_marker.pose.orientation.x = 0.0;   
      arrow_marker.pose.orientation.y = 0.0;   
      arrow_marker.pose.orientation.z = 1.0;
      arrow_marker.pose.orientation.w = 1.0;      

      int_marker_arrow.name = Y_SIZE_ARROW_1;
      size_arrow_control.name = "move_y";
  }else if ( arrow_name.compare(Z_SIZE_ARROW_1) == 0) {
      // Set initial interactive marker pose    
      int_marker_arrow.pose.position.z = box_size_in_dimension/2.0;
      
      // Set Arrow Orientation
      arrow_marker.pose.orientation.x = 0.0;   
      arrow_marker.pose.orientation.y = 1.0;   
      arrow_marker.pose.orientation.z = 0.0;
      arrow_marker.pose.orientation.w = -1.0;      

      int_marker_arrow.name = Z_SIZE_ARROW_1;
      size_arrow_control.name = "move_z";
  }
  // Set the orientation of the control direction
  size_arrow_control.orientation = arrow_marker.pose.orientation;

  // Specify the control properties
  size_arrow_control.always_visible = true;
  size_arrow_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  // Use the arrow marker as the draggable object
  size_arrow_control.markers.push_back(arrow_marker);

  // Add the control to the arrow interactive marker
  int_marker_arrow.controls.push_back(size_arrow_control);    
}

void updateArrowLocations(const geometry_msgs::Pose box_pose, 
                          const visualization_msgs::Marker box_marker){
  // Update location of x arrow 
  geometry_msgs::Pose pose_arrow_x = box_pose;
  pose_arrow_x.position.x = pose_arrow_x.position.x + box_marker.scale.x/2.0;
  server->setPose(X_SIZE_ARROW_1, pose_arrow_x);

  // Update location of y arrow 
  geometry_msgs::Pose pose_arrow_y = box_pose;
  pose_arrow_y.position.y = pose_arrow_y.position.y + box_marker.scale.y/2.0;
  server->setPose(Y_SIZE_ARROW_1, pose_arrow_y);  

  // Update location of z arrow
  geometry_msgs::Pose pose_arrow_z = box_pose;
  pose_arrow_z.position.z = pose_arrow_y.position.z + box_marker.scale.z/2.0;
  server->setPose(Z_SIZE_ARROW_1, pose_arrow_z);

  server->applyChanges();
}


void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y  << ", " << feedback->pose.position.z );

  visualization_msgs::InteractiveMarker int_box_marker;
  visualization_msgs::Marker box_marker_copy;

  server->get(MARKER_NAME, int_box_marker);
  box_marker_copy = int_box_marker.controls[0].markers[1];

  updateArrowLocations(feedback->pose, box_marker_copy);
}

void processFeedbackArrow(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  ROS_INFO_STREAM( feedback->marker_name << " is now at "<< feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z );

  visualization_msgs::InteractiveMarker int_box_marker;
  visualization_msgs::Marker box_marker_copy;

  // Get a copy of the Interactive box marker
  server->get(MARKER_NAME, int_box_marker);
  box_marker_copy = int_box_marker.controls[0].markers[1];  

  // Change the size of the box depending on the arrow type
  if ( (feedback->marker_name).compare(X_SIZE_ARROW_1) == 0 ){
    std::cout << "box marker x scale:" << int_box_marker.controls[0].markers[1].scale.x << std::endl;
    box_marker_copy.scale.x = 2.0*(feedback->pose.position.x - int_box_marker.pose.position.x);    
  }else if( (feedback->marker_name).compare(Y_SIZE_ARROW_1) == 0) {
    std::cout << "box marker y scale:" << int_box_marker.controls[0].markers[1].scale.y << std::endl;
    box_marker_copy.scale.y = 2.0*(feedback->pose.position.y - int_box_marker.pose.position.y);
  }else if( (feedback->marker_name).compare(Z_SIZE_ARROW_1) == 0) {
    std::cout << "box marker z scale:" << int_box_marker.controls[0].markers[1].scale.z << std::endl;
    box_marker_copy.scale.z = 2.0*(feedback->pose.position.z - int_box_marker.pose.position.z);
  }

  // Update the interactive box marker with a new size
  int_box_marker.controls[0].markers[1] = box_marker_copy;
  server->insert(int_box_marker);
  server->applyChanges();
}

bool getCloudinBox(){
  if (!cloud->points.size() > 0){
    ROS_ERROR("point cloud is empty");
    return false;
  }

  // Clear the points
  boxed_cloud->points.clear();  
  PointCloudRGB::Ptr cloud_in_world_frame (new PointCloudRGB);  

  tf::StampedTransform transform;
  int attempts = 0;
  while (ros::ok()){
      try{
          if (attempts == 10){
            ROS_ERROR("Cannot Get transform for the pointcloud after 10 attempts");
            return false;
            break;
          }
        // Look up transform
        tf_listener->lookupTransform(MARKER_FRAME, cloud->header.frame_id, ros::Time(0), transform);
        pcl_ros::transformPointCloud((*cloud), (*cloud_in_world_frame), transform);          
      break;
      }
      //keep trying until we get the transform
      catch (tf::TransformException &ex){
        ROS_ERROR_THROTTLE(2,"%s",ex.what());
        ROS_WARN_THROTTLE(2, "   Waiting for tf to transform desired SAC axis to point cloud frame. trying again");
        ros::Duration(1.0).sleep();
        attempts++;
      }
  }

  // Grab the state of the bounding box
  visualization_msgs::InteractiveMarker int_box_marker;
  visualization_msgs::Marker box;
  geometry_msgs::Pose box_pose;
  server->get(MARKER_NAME, int_box_marker);


  // Create copies
  box = int_box_marker.controls[0].markers[1];  
  box_pose = int_box_marker.pose;

  // Set axis-aligned min and max for the filter
  double x_min = (box_pose.position.x - (box.scale.x/2.0));
  double x_max = (box_pose.position.x + (box.scale.x/2.0));  

  double y_min = (box_pose.position.y - (box.scale.y/2.0));
  double y_max = (box_pose.position.y + (box.scale.y/2.0));  

  double z_min = (box_pose.position.z - (box.scale.z/2.0));
  double z_max = (box_pose.position.z + (box.scale.z/2.0));  

  // Create Containers for each filter iteration
  PointCloudRGB::Ptr x_filtered (new PointCloudRGB);
  PointCloudRGB::Ptr y_filtered (new PointCloudRGB);  
  std::cout <<  "Number of initial points " <<  cloud_in_world_frame->points.size() << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass (true);//////////////////////////damn////////////////////////////////////////////////////////////////////////////////////////////////
  // Filter x dimension:
  pass.setInputCloud (cloud_in_world_frame);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_min , x_max);
  pass.filter (*x_filtered);

  // Filter y dimension:
  pass.setInputCloud (x_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_min , y_max);
  pass.filter (*y_filtered);

  // Filter z dimension:
  pass.setInputCloud (y_filtered);  
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_min , z_max);
  pass.filter (*boxed_cloud);  /*
  for (int ind = 0; ind < cloud_in_world_frame->points.size() ;ind++)
  {
    pcl::PointXYZRGB pt = cloud_in_world_frame->points[ind];
    if (pt.x > x_min && pt.x < x_max && pt.y > y_min && pt.y < y_max && pt.z < z_max && pt.z > z_min)
      boxed_cloud->points.push_back(pt);
  }*/
  std::cout <<  "Number of points " <<  boxed_cloud->points.size() << std::endl;

  // Update the header
  boxed_cloud->header = (*cloud).header; 
  boxed_cloud->header.frame_id = MARKER_FRAME;
  std::cout << "Frame to publish: " <<  boxed_cloud->header.frame_id << std::endl;

  ROS_INFO("Filtered Cloud has: %zu points", boxed_cloud->points.size());
/*  pcl::PointXYZRGB pt;
  double sumx = 0;
  double sumy = 0;
  for (int i=0; i<boxed_cloud->points.size();i++)
  {
    pt = boxed_cloud->points[i];
    sumx = sumx + pt.x;
    sumy = sumy + pt.y;
  }

  double offsetx = sumx / boxed_cloud->points.size();
  double offsety = sumy / boxed_cloud->points.size();
  std::cout << "Mean of x is "<< offsetx << std::endl;
  std::cout << "Mean of y is "<< offsety << std::endl;
  for (int i=0; i<boxed_cloud->points.size();i++)
  {
    boxed_cloud->points[i].x = boxed_cloud->points[i].x - offsetx;
    boxed_cloud->points[i].y = boxed_cloud->points[i].y - offsety;
  }*/
  pointcloud_pub.publish(boxed_cloud);
  return true;
}

void getRelativePose(std::string hand_side){ 
}

bool loadStoredCloud(){
  PointCloudRGB::Ptr  stored_pc(new PointCloudRGB);
  std::string package_path = ros::package::getPath("object_registration");
  std::string savepath = package_path + "/pcd_saved_files/";
  try{
    // Save Transformed Cloud
    ros::NodeHandle private_nh("~");      
    std::string load_filename="door.pcd";
/*    private_nh.param<std::string>("load_filename", load_filename, DEFAULT_PC_NAME);
*/
    pcl::io::loadPCDFile(savepath + load_filename, *stored_pc);///////////////////////////////////////////////////////////////////////////////////////////////////////
    ROS_INFO("Successfully loaded the point cloud");
    ROS_INFO("Visualizing on the topic %s", stored_cloud_topic.c_str());      
    
    stored_pc->header.frame_id = MARKER_FRAME;
    stored_pc->header.stamp = ros::Time::now().toNSec() / 1000ull; // Convert from ns to us
    storedcloud_pub.publish(stored_pc);
  }
    //keep trying until we get the transform
  catch (pcl::IOException& pcl_ex){//(...){
    ROS_ERROR("Caught PCL exception. Cannot open the directory to save the file");
    return false;
  }
  return true;
}

bool storeCloudInfo(){
    if (!cloud->points.size() > 0){
      ROS_ERROR("point cloud is empty");
      return false;
    }  
    std::string package_path = ros::package::getPath("object_registration");
    std::string savepath = package_path + "/pcd_saved_files/";

    try{
      ros::NodeHandle private_nh("~");      
      std::string save_filename;
      private_nh.param<std::string>("save_filename", save_filename, DEFAULT_PC_NAME);

      // Save Transformed Cloud
      pcl::io::savePCDFileASCII (savepath + save_filename, *boxed_cloud);////////////////////////////////////////////////////////////////////////////////////////
      ROS_INFO("Successfully saved the point clouds in the boxed region");
    }
      //keep trying until we get the transform
    catch (pcl::IOException& pcl_ex){//(...){
      ROS_ERROR("Caught PCL exception. Cannot open the directory to save the file");
      return false;
    }

    return true;

}


void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  cloud_mutex.lock();  // Lock Mutex
  pcl::fromROSMsg(*msg, *cloud);   // Perform Copy
  cloud_mutex.unlock();   // Unlock Mutex

  ROS_INFO("Pointcloud has %zu points", cloud->points.size());
  //getCloudinBox();
}

void command_callback(const std_msgs::String::ConstPtr& msg){
  std::string get_cloud_in_box;     get_cloud_in_box = "get_cloud_in_box";
  std::string store_cached_cloud;   store_cached_cloud = "store_cached_cloud";  
  std::string load_object;          load_object = "load_object";    

  if (get_cloud_in_box.compare(msg->data) == 0){
    ROS_INFO("Getting Cloud within the Box");
    getCloudinBox();  
  }else if (store_cached_cloud.compare(msg->data) == 0){
    ROS_INFO("Storing Cloud within the Box");
    storeCloudInfo();      
  } else if(load_object.compare(msg->data) == 0){
    ROS_INFO("Loading the stored cloud");
    loadStoredCloud();
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_registration_interest_box");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Get ROS params
/*  std::string point_cloud_topic_name;
  private_nh.param<std::string>("point_cloud_topic_name", point_cloud_topic_name, DEFAULT_POINTCLOUD_SUB);
  std::cout << "Subscribing to " << point_cloud_topic_name << std::endl;
  */

  // Declare Subscribers
  ros::Subscriber  pointcloud_sub;
  ros::Subscriber  operator_command_sub;

  pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(DEFAULT_POINTCLOUD_SUB, 10, cloud_callback);    
  operator_command_sub = nh.subscribe<std_msgs::String>("gui_object_registration_manager/operator_command", 1, command_callback);

  // Declare Publishers
  pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(POINTCLOUD_PUB_NAME, 0);
  storedcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(STORED_CLOUD_PUB_NAME, 0);

  tf_listener = new tf::TransformListener;

  //register ctrl-c
  signal(SIGINT, sig_handler);

  // Set the interactive marker server on the topic namespace simple_marker
  server.reset( new interactive_markers::InteractiveMarkerServer("simple_marker","",false) );

  // Create the Interactive Box Marker which will indicate the region of interest
  visualization_msgs::InteractiveMarker int_marker;
  visualization_msgs::Marker box_marker_copy;
  createBoxMarker(int_marker, box_marker_copy);

  // Create the Interactive Arrow Markers which changes the size of the box
  visualization_msgs::InteractiveMarker int_marker_arrow_x;
  visualization_msgs::InteractiveMarker int_marker_arrow_y;
  visualization_msgs::InteractiveMarker int_marker_arrow_z;
  createArrowMarker(X_SIZE_ARROW_1, box_marker_copy.scale.x, int_marker_arrow_x);
  createArrowMarker(Y_SIZE_ARROW_1, box_marker_copy.scale.y, int_marker_arrow_y);
  createArrowMarker(Z_SIZE_ARROW_1, box_marker_copy.scale.z, int_marker_arrow_z);  

  // Insert to the Server
  server->insert(int_marker, &processFeedback);
  server->insert(int_marker_arrow_x, &processFeedbackArrow);  
  server->insert(int_marker_arrow_y, &processFeedbackArrow);    
  server->insert(int_marker_arrow_z, &processFeedbackArrow);      

  // 'commit' changes and send to all clients
  server->applyChanges();



  // start the ROS main loop
  ros::spin();
}
