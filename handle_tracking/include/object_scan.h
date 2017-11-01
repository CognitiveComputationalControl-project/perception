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


	ros::Publisher Gaze_point_pub;
	ros::Publisher Gaze_activate_pub;
	ros::Publisher setNavTarget_pub;
	
	int index;
	tf::TransformListener 	  listener;

	std::vector<double> Robot_Pos;				//x,y,theta
	std::vector<double> Head_Pos;				//x,y,theta
	std::vector<double> global_pose;
	
	visualization_msgs::MarkerArray human_boxes_array;
	bool OnceTargeted;

	// void number_human_callback(const )
	void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void Human_MarkerarrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
	void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
	void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void keyboard_callback(const keyboard::Key::ConstPtr& msg);
	
	
	void getCameraregion();
	bool getlinevalue(int line_type,double input_x, double input_y);
	// void setViewpointTarget(const std::vector<double> pos);
		

};