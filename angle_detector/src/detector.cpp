#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <boost/thread/mutex.hpp>


using namespace std;

bool sensor_on   = false;

int g_counter = 0;

vector < double > rec_x;
vector < double > rec_y;
string sensor_frame_id;
sensor_msgs::LaserScan SensorMsg;
boost::mutex mutex;

void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg);

// added by Ferdian Jovan
// function to restrict a possibility of persons standing next to each other


int main(int argc, char **argv){

  ros::init(argc, argv, "angle_detector");
  ros::NodeHandle n;
  ros::Publisher  node_pub = n.advertise <std_msgs::Float32>("angle_detector", 2); // Humans in the environment
  // get param from launch file
  string laser_scan = "/hsrb/base_scan";
  ros::param::get("~laser_scan", laser_scan);
  ros::Subscriber node_sub = n.subscribe(laser_scan, 2, LaserCallback);
  geometry_msgs::PoseArray msgx;

  ros::Rate loop_rate(50);
  std_msgs::Float32 angle;
  bool door_first_found = false;
  while( ros::ok() ){
    if( sensor_on == true  && door_first_found == false){
      // find door : push the data into a pointcloud & use RANSAC to fit a line (http://pointclouds.org/documentation/tutorials/random_sample_consensus.php)

      // save door


      //door_found = true;

      door_first_found = true;
    }
    else if( sensor_on == true  && door_first_found == true){
      // update door

      // publish door
      node_pub.publish( angle );
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg){

/*
  // To get header data from sensor msg
  SensorMsg = *msg;

  // Vectors...
  rec_x.clear(); 
  rec_y.clear(); 
  
  sensor_on = true;
  
  double px, py, pr, pt;
  vector < double >  laser_x;
  vector < double >  laser_y;
  vector < double >  laser_r;
  vector < double >  laser_t;
  for( unsigned i = 0; i < msg->ranges.size(); i++ ){    
    pr = msg->ranges[ i ];
    pt = msg->angle_min + ( i * msg->angle_increment);
    laser_r.push_back( pr );
    laser_t.push_back( pt );
  }

//  float32 angle_min        # start angle of the scan [rad]
//  float32 angle_max        # end angle of the scan [rad]
//  float32 angle_increment  # angular distance between measurements [rad]
//  As an example, to access one of these values, write : msg->angle_min

//  You are only interested in the points which are between two angles,
<<<<<<< HEAD
  //for( unsigned i = [> starting angle ; i < /* last angle <]; i++ ) 
  //{    
      /*Save points*/
  //}
	

 /* for( unsigned i =  starting angle ; i < /* last angle *; i++ ) 
  {    
      /*Save points
  }
	
*/
}
