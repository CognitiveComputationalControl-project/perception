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


class state_planner{

public:
	state_planner();
	~state_planner();


    ros::Publisher states_pub;
    ros::Publisher action_pub;
	ros::Publisher cmd_pub;
	ros::ServiceClient client;

	int index;
	tf::TransformListener 	  listener;
	int current_state;

	
	void update_state();
	void set_state();
	void print_state();
	// void Publish_visualized_marker(const geometry_msgs::PoseStamped Pose);
		

};
