#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseArray.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <keyboard/Key.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include "handle_detector/localize_handle.h"
#include "handle_tracking/objectfinder.h"



#define FOVW 29				//field of view width
#define MATH_PI 3.14159265359
#define P_H 0.2 // Prior prob
#define P_S_given_H 0.8
#define P_S_given_Hc 0.5

#define P_Sc_given_H 0.01
#define P_Sc_given_Hc 0.99

#define Same_POS_diff 0.5
#define MAX_UPDATE_ITER 50
#define MAX_VIEW_UPDATE_ITER 100
#define LASER_ANGLE_RES 0.25
#define LASER_Data_Length 914
#define LASER_Point_Step 16
#define LASER_Dist_person 4.0


#define LASER_ANGLE_MIN -2.09875845909
#define LASER_ANGLE_MAX 2.09875845909
#define LASER_ANGLE_STEP 0.00436332309619



class Handle_manager{

public:
	Handle_manager();
	Handle_manager(int numofhuman);
	~Handle_manager();

/*
	ros::Publisher Gaze_point_pub;
	ros::Publisher Gaze_activate_pub;
	ros::Publisher setNavTarget_pub;*/
    ros::Publisher handletarget_pub;
    ros::Publisher handlemiddletarget_pub;
	ros::Publisher grasp_pub;
	ros::Subscriber sub;
	ros::ServiceClient client;
	ros::ServiceServer service;
  	ros::NodeHandle n;
	#define MAP_FRAME "map"
	#define RANGE_SENSOR_FRAME "head_rgbd_sensor_rgb_frame"
	#define BASE_FRAME "base_link"
    std::string MARKER_FRAME;
	visualization_msgs::MarkerArray marker_update;
	int index;
	tf::TransformListener 	  listener;
	handle_detector::localize_handle srv;
	handle_tracking::objectfinder srv_find;
	std::vector<double> Robot_Pos;				//x,y,theta
	std::vector<double> Head_Pos;				//x,y,theta
	std::vector<double> global_pose;
	double y_left;
	geometry_msgs::PoseStamped grasp_pose;
	geometry_msgs::PoseStamped grasp_transformed_pose;
	visualization_msgs::MarkerArray human_boxes_array;
	bool OnceTargeted;

	void marker_sorting(const visualization_msgs::MarkerArray msg);
	void set_marker(const visualization_msgs::MarkerArray markersrv);
	void Publish_visualized_marker(const geometry_msgs::PoseStamped Pose);
	void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input);
	bool track_handle(handle_tracking::objectfinder::Request  &req,handle_tracking::objectfinder::Response &res);

};
