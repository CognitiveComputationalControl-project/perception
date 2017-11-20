#include "object_scan.h"
#include "handle_detector/localize_handle.h"
#include "handle_tracking/objectfinder.h"

Handle_manager::~Handle_manager(){}
Handle_manager::Handle_manager()
{
    x_left= 1000;
    
}

void Handle_manager::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  srv.request.pointcloud_data = *input;
}

bool Handle_manager::track_handle(handle_tracking::objectfinder::Request  &req,
         handle_tracking::objectfinder::Response &res)
{
  ros::Rate loop_rate(10);
  int found = 0;
  while (ros::ok())
  {
  if (found > 10){
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
        found+=1;
        std::cout<<found<<std::endl;
      }
    }
  } 
  ros::spinOnce();
  loop_rate.sleep();  
  }
  return true;
}

void Handle_manager::set_marker(const visualization_msgs::MarkerArray markersrv)
{
    marker_update = markersrv;
    marker_sorting(marker_update);
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

    double temp_dist=0.050;

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
    ROS_INFO("sorting started");
    ROS_INFO("%d", marker_update.markers.size());
    for (int i = 0 ; i < marker_update.markers.size(); i++)
    {
      if ( (x_left > marker_update.markers[i].pose.position.x) /*&& abs(msg->markers[i].pose.position.y)< 0.2*/)
        { 
        x_left = marker_update.markers[i].pose.position.x;
        grasp_pose.pose=marker_update.markers[i].pose;
        grasp_pose.header =marker_update.markers[i].header;
        ROS_INFO("New minimum value found : %f", x_left);
        }
    }

    grasp_pose.header.stamp = ros::Time::now();
    grasp_pose.header.frame_id = "head_rgbd_sensor_rgb_frame";

    //transform
    tf::StampedTransform transform_sensor_base;
    listener.waitForTransform("head_rgbd_sensor_rgb_frame","odom",  ros::Time(0), ros::Duration(3.0));
    while (ros::ok()){
    try{ 
        listener.lookupTransform("head_rgbd_sensor_rgb_frame","odom", ros::Time(0), transform_sensor_base);
        listener.transformPose (std::string("odom"), grasp_pose,  grasp_transformed_pose) ; 
        ROS_INFO("_x : %.3lf, _y : %.3lf, _z : %.3lf \n ", grasp_transformed_pose.pose.position.x, grasp_transformed_pose.pose.position.y, grasp_transformed_pose.pose.position.z) ; 
        break;
    }   
    catch (tf::TransformException &ex){
    }
    }
    grasp_transformed_pose.header.stamp=ros::Time::now();
    grasp_transformed_pose.header.frame_id="odom";
    Publish_visualized_marker(grasp_transformed_pose);
    grasp_pub.publish(grasp_transformed_pose);

}
