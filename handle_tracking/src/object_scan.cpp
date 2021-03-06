#include "object_scan.h"
#include "handle_detector/localize_handle.h"
#include "handle_tracking/objectfinder.h"
#include <string>
#include <iostream>
Handle_manager::~Handle_manager(){}
Handle_manager::Handle_manager()
{
    y_left= -1000;
    
}

void Handle_manager::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  srv.request.pointcloud_data = *input;
}

bool Handle_manager::track_handle(handle_tracking::objectfinder::Request  &req,
handle_tracking::objectfinder::Response &res)
{
  handle_found = false;
  sub = n.subscribe<sensor_msgs::PointCloud2>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 
                                                                       10, &Handle_manager::cloud_callback,this);
  ros::Rate loop_rate(10);
  int found = 0;
  while (ros::ok())
  {
  if (found > 2){
    ROS_INFO("breaking");
    break;
  }
  else if(srv.request.pointcloud_data.data.size()>0)
  {
    ROS_INFO("pointcloud passed");

   if (client.call(srv))
    {
      if (srv.response.handle_marker.markers.size()!=0)
      {
        ROS_INFO("marker received");
        set_marker(srv.response.handle_marker);
        marker_sorting(marker_update);
        found+=1;
        std::cout<<found<<std::endl;
      }
    }
  } 
  ros::spinOnce();
  loop_rate.sleep();  
  }
  res.handle_is_found = handle_found;
  res.best_grasp_pose = grasp_transformed_pose;
  sub.shutdown();
  return true;
}

void Handle_manager::set_marker(const visualization_msgs::MarkerArray markersrv)
{
    marker_update = markersrv; // frame_id -> head
}

void Handle_manager::Publish_visualized_marker(const geometry_msgs::PoseStamped Pose)
{

    visualization_msgs::Marker visual_marker; 
    visual_marker.header.frame_id = Pose.header.frame_id; 
    visual_marker.header.stamp = ros::Time::now();
    visual_marker.id = 0;
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    visual_marker.type = shape;

    visual_marker.pose.position.x = Pose.pose.position.x;
    visual_marker.pose.position.y = Pose.pose.position.y;
    visual_marker.pose.position.z = Pose.pose.position.z;

    visual_marker.pose.orientation.x = 0.0;
    visual_marker.pose.orientation.y = 0.0;
    visual_marker.pose.orientation.z = 0.0;
    visual_marker.pose.orientation.w = 1.0;

    double temp_dist=0.070;

    //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
    visual_marker.scale.x = std::abs(temp_dist);
    visual_marker.scale.y = std::abs(temp_dist);
    visual_marker.scale.z = std::abs(temp_dist);

    visual_marker.color.r = 0.0;
    visual_marker.color.g = 0.7;
    visual_marker.color.b = 0.2;
    visual_marker.color.a = 0.85;

    handletarget_pub.publish(visual_marker);

}



void Handle_manager::marker_sorting(const visualization_msgs::MarkerArray msg)
{
  ROS_INFO("%d marker candidates", marker_update.markers.size());
  for (int i = 0 ; i < marker_update.markers.size(); i++)
  {

    grasp_pose.header.stamp = ros::Time::now();
    grasp_pose.header.frame_id = RANGE_SENSOR_FRAME;
    grasp_pose.pose = marker_update.markers[i].pose;
    grasp_pose.header = marker_update.markers[i].header;
    //transform
    tf::StampedTransform transform_sensor_base;
    listener.waitForTransform(RANGE_SENSOR_FRAME,MAP_FRAME,  ros::Time(0), ros::Duration(3.0));
    int ite = 0;
    while (ros::ok() && ite < 5)
    {
      try
      { 

        listener.lookupTransform(RANGE_SENSOR_FRAME,MAP_FRAME, ros::Time(0), transform_sensor_base);
        //listener.transformPose(MAP_FRAME, grasp_pose,  grasp_transformed_pose);
        listener.transformPose (MAP_FRAME, grasp_pose,  temp_grasp_transformed_pose) ;
        temp_grasp_transformed_pose.header.stamp=ros::Time::now();
        temp_grasp_transformed_pose.header.frame_id=MAP_FRAME;
        /*          ROS_INFO("%d is the temp_grasp_transformed_pose.pose.position.y", temp_grasp_transformed_pose.pose.position.y);
                    ROS_INFO("%d is the temp_grasp_transformed_pose.pose.position.z", temp_grasp_transformed_pose.pose.position.z);*/

        if ( (y_left < temp_grasp_transformed_pose.pose.position.y) && (fabs(temp_grasp_transformed_pose.pose.position.z-0.93)<0.02) )
        { 
          ROS_INFO("_x : %.3lf, _y : %.3lf, _z : %.3lf \n ", temp_grasp_transformed_pose.pose.position.x, temp_grasp_transformed_pose.pose.position.y, temp_grasp_transformed_pose.pose.position.z) ; 
          Publish_visualized_marker(temp_grasp_transformed_pose);
          y_left = temp_grasp_transformed_pose.pose.position.y;
          grasp_transformed_pose =  temp_grasp_transformed_pose;
          handle_found = true;
          break;
        }
      }   
      catch (tf::TransformException &ex){
      }
      ite = ite +1;
    }
/*    std::cout << "considered candidates are : " <<grasp_transformed_pose.pose.position.z << " and " << fabs(grasp_transformed_pose.pose.position.z-0.94)<<std::endl;
*/
  }
}
