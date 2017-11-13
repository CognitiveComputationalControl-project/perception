#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <sstream>
#include <string>
#include <ostream>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"

using namespace std;

int dimcloud=0;
bool sensor_on   = false;
int g_counter = 0;

vector < double >  laser_x;
vector < double >  laser_y;
vector < double >  laser_r;
vector < double >  laser_t;

string sensor_frame_id;

sensor_msgs::LaserScan SensorMsg;

boost::mutex mutex;


boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}





void RanSac(pcl::PointCloud<pcl::PointXYZ>::Ptr final);

void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg);


// vector < double > rec_x;
// vector < double > rec_y;
// string sensor_frame_id;
// sensor_msgs::LaserScan SensorMsg;
// boost::mutex mutex;

void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg);

// added by Ferdian Jovan
// function to restrict a possibility of persons standing next to each other



int main(int argc, char **argv){

  ros::init(argc, argv, "angle_detector");
  ros::NodeHandle n;

  ros::Publisher  angle_detector_pub = n.advertise <std_msgs::Float32>("angle_detector", 2);
  ros::Publisher  AnglePoint_ref_pub=n.advertise <visualization_msgs::MarkerArray>("anglepoint_ref_markerarray", 2);
  ros::Publisher  AnglePoint_cur_pub=n.advertise <visualization_msgs::MarkerArray>("anglepoint_cur_markerarray", 2);

  visualization_msgs::MarkerArray ref_markerarray;
  visualization_msgs::MarkerArray cur_markerarray;

  visualization_msgs::Marker ref_marker;

  string laser_scan = "/hsrb/base_scan";
  ros::param::get("~laser_scan", laser_scan);
  ros::Subscriber node_sub = n.subscribe(laser_scan, 10, LaserCallback);



  geometry_msgs::PoseArray msgx;

  ros::Rate loop_rate(50);
  std_msgs::Float32 angle;

  bool door_first_found = false;
  int index = 1;

  //definition of pointclouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_closed (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_open (new pcl::PointCloud<pcl::PointXYZ>);

  // publisher of the current pointcloud
  ros::Publisher  pub ;
  pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> >("pointcloud_irene", 1);

  sensor_msgs::PointCloud2 PCLD;
  sensor_msgs::PointCloud2 PCLD2;

  // vector parallel to the door
  std::vector<double> closed_door_vec;
  std::vector<double> open_door_vec;


 while( ros::ok() ){

    if( sensor_on == true  && door_first_found == false){
       // find door : push the data into a pointcloud & use RANSAC to fit a line 
	     // initialize PointClouds 
	     RanSac(final_closed);
       // http://pointclouds.org/documentation/tutorials/random_sample_consensus.php

       final_closed->header.frame_id = "base_range_sensor_link";
       final_closed->height = 1;
       final_closed->width = dimcloud;

       //msg->points.push_back (final_closed);
       pcl::toROSMsg(*final_closed, PCLD);
       pub.publish(PCLD);

       // create the vector that follows the door direction 
       closed_door_vec.resize(2);
       closed_door_vec[0]=laser_x[dimcloud-1]-laser_x[1];
       closed_door_vec[1]=laser_y[dimcloud-1]-laser_y[1];

  
        visualization_msgs::Marker visual_marker; 

        visual_marker.header.frame_id = "base_range_sensor_link"; 
        visual_marker.header.stamp = ros::Time::now();
        visual_marker.id = 0;
        uint32_t shape = visualization_msgs::Marker::SPHERE;
        visual_marker.type = shape;

        visual_marker.pose.position.x = laser_x[1];
        visual_marker.pose.position.y = laser_y[1];
        visual_marker.pose.position.z = 0.3;

        visual_marker.pose.orientation.x = 0.0;
        visual_marker.pose.orientation.y = 0.0;
        visual_marker.pose.orientation.z = 0.0;
        visual_marker.pose.orientation.w = 1.0;

        double temp_dist=0.050;

        //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
        visual_marker.scale.x = std::abs(temp_dist);
        visual_marker.scale.y = std::abs(temp_dist);
        visual_marker.scale.z = std::abs(temp_dist);

        visual_marker.color.r = 0.0;
        visual_marker.color.g = 0.7;
        visual_marker.color.b = 0.2;
        visual_marker.color.a = 0.85;

        ref_markerarray.markers.push_back(visual_marker);

        visual_marker.id=1;
        visual_marker.pose.position.x = laser_x[dimcloud-1];
        visual_marker.pose.position.y = laser_y[dimcloud-1];
        visual_marker.pose.position.z = 0.3;

        visual_marker.color.r = 0.85;
        visual_marker.color.g = 0.7;
        visual_marker.color.b = 0.2;
        visual_marker.color.a = 0.85;

        ref_markerarray.markers.push_back(visual_marker);
       //ROS_INFO("point 1 : %lf, %lf \n", laser_x[1],laser_y[1]);
       //ROS_INFO("point 2 : %lf, %lf \n", laser_x[dimcloud-1],laser_y[dimcloud-1]);

       //WE SHOULD MAKE HIM TALK AND SAY ("CLOSED DOOR DETECTED")
       //ROS_INFO("closed door vector is : %f, %f \n", closed_door_vec[0],closed_door_vec[1]);
       door_first_found = true;

    }

    else if( sensor_on == true  && door_first_found == true){
      // update door
       //rospy.sleep(1.)
       RanSac(final_open);

       final_open->header.frame_id = "base_range_sensor_link";
       final_open->height = 1;
       final_open->width = dimcloud;

       //msg->points.push_back (final_closed);
       pcl::toROSMsg(*final_open, PCLD2);
       pub.publish(PCLD2);

       // create the vector that follows the door direction 
       

       open_door_vec.resize(2);
       open_door_vec[0]=laser_x[dimcloud-1]-laser_x[1];
       open_door_vec[1]=laser_y[dimcloud-1]-laser_y[1];

       cur_markerarray.markers.clear();

        visualization_msgs::Marker visual_marker; 

        visual_marker.header.frame_id = "base_range_sensor_link"; 
        visual_marker.header.stamp = ros::Time::now();
        visual_marker.id = 0;
        uint32_t shape = visualization_msgs::Marker::SPHERE;
        visual_marker.type = shape;

        visual_marker.pose.position.x = laser_x[1];
        visual_marker.pose.position.y = laser_y[1];
        visual_marker.pose.position.z = 0.3;

        visual_marker.pose.orientation.x = 0.0;
        visual_marker.pose.orientation.y = 0.0;
        visual_marker.pose.orientation.z = 0.0;
        visual_marker.pose.orientation.w = 1.0;

        double temp_dist=0.050;

        //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
        visual_marker.scale.x = std::abs(temp_dist);
        visual_marker.scale.y = std::abs(temp_dist);
        visual_marker.scale.z = std::abs(temp_dist);

        visual_marker.color.r = 0.85;
        visual_marker.color.g = 0.2;
        visual_marker.color.b = 0.7;
        visual_marker.color.a = 0.85;

        cur_markerarray.markers.push_back(visual_marker);

        visual_marker.id=1;
        visual_marker.pose.position.x = laser_x[dimcloud-1];
        visual_marker.pose.position.y = laser_y[dimcloud-1];
        visual_marker.pose.position.z = 0.2;

        visual_marker.color.r = 0.85;
        visual_marker.color.g = 0.1;
        visual_marker.color.b = 0.2;
        visual_marker.color.a = 0.85;

        cur_markerarray.markers.push_back(visual_marker);

       //ROS_INFO("point 1 : %lf, %lf \n", laser_x[1],laser_y[1]);
       //ROS_INFO("point 2 : %lf, %lf \n", laser_x[dimcloud-1],laser_y[dimcloud-1]);
       //ROS_INFO("open door vector is : %f, %f \n", open_door_vec[0],open_door_vec[1]);

      // publish door
      //angle_detector_pub.publish( angle );
       double dot=0.0;
       double det=0.0;
       double angle=0.0;
       

       dot = closed_door_vec[0]*open_door_vec[0] + closed_door_vec[1]*open_door_vec[1];
       det = closed_door_vec[0]*open_door_vec[1] - closed_door_vec[1]*open_door_vec[0];
       angle = static_cast<double> (atan2(det, dot)*360/3.141592);
       printf("the angle is : %.3lf \n",angle);
    }

    AnglePoint_cur_pub.publish(cur_markerarray);
    AnglePoint_ref_pub.publish(ref_markerarray);
    ros::spinOnce();
    loop_rate.sleep();
  }
  

  ros::spin();

  return 0;
}


void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg){

  //ROS_INFO("laser callback");

  // To get header data from sensor msg
  SensorMsg = *msg;

  // Vectors...
  laser_x.clear(); 
  laser_y.clear(); 
  laser_r.clear(); 
  laser_t.clear(); 
    
  sensor_on = true;
  
  double px, py, pr, pt, k;
  k = 0;

  for( unsigned i = 0; i < msg->ranges.size(); i++ ){    
    pr = msg->ranges[ i ];
    pt = msg->angle_min + ( i * msg->angle_increment);
    px = pr * cos (pt);
    py = pr * sin(pt); 

    if (  (pt < 0.80) && (pt > -0.10) )
    {
       if (  (px-k) < 3   )  
       {

       k=0;
       laser_x.push_back( px );
       laser_r.push_back( pr );
       laser_t.push_back( pt );       
       laser_y.push_back( py );
       k = px;

       }

    }
    
  }

  
  dimcloud = laser_r.size();
  //std::cout<< msg->angle_min << std::endl;
}


void RanSac(pcl::PointCloud<pcl::PointXYZ>::Ptr final){
 
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
 cloud->width    = dimcloud;
 cloud->height   = 1;

 //ROS_INFO("cloud width : %d, dimcloud : %d,  heigth : %d\n",cloud->width, dimcloud, cloud->height);

 //ROS_INFO("laser x, y size : %d, %d \n", laser_x.size(),laser_y.size());
 cloud->is_dense = false;
 cloud->points.resize (cloud->width * cloud->height);

 for (size_t i = 0; i < cloud->points.size (); ++i)
 {
 	     cloud->points[i].x = laser_x.at(i);
       cloud->points[i].y = laser_y.at(i);
       cloud->points[i].z = 0.2;
 }

 std::vector<int> inliers;

 pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
 model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

 pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
 ransac.setDistanceThreshold (.01);
 ransac.computeModel();
 ransac.getInliers(inliers); 
// ROS_INFO("I am here");
//for (size_t i = 1; i < cloud->points.size (); ++i)
 //{
  //     if (cloud->points[i].x) 

   //     cloud->points[i-1].x - cloud->points[i] > laser_x.at(i);
    //   cloud->points[i].y = laser_y.at(i);
     //  cloud->points[i].z = 0.2;
 //}
 // copies all inliers of the model computed to another PointCloud
 pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

 //ROS_INFO("I am here 2");
 // // creates the visualization object and adds either our orignial cloud or all of the inliers
 // // depending on the command line arguments specified.
 // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
 
 // viewer = simpleVis(final_closed);
 
 // while (!viewer->wasStopped ())
 // {
 //    	viewer->spinOnce (100);
 //    	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 // }
     
return;

}



