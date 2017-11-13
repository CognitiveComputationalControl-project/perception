#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "door_recognition/ObjectLocalize.h"
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/impl/transforms.hpp>


#include "quat_helper.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_broadcaster.h>
tf::TransformListener *tf_listener;

typedef pcl::PointXYZRGB PointNT;

typedef pcl::PointCloud<PointNT> PointCloudT;
sensor_msgs::PointCloud2 object_transformed_cloud_msg;    

ros::ServiceClient door_detector_client;
door_recognition::ObjectLocalize srv; 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  stored_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  scene_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
ros::ServiceClient object_localize_client;
pcl::PointCloud<pcl::PointXYZ>::Ptr  handle_pcl(new pcl::PointCloud<pcl::PointXYZ>);

    ros::Publisher object_trans_cloud_pub;
    ros::Publisher door_in_baselink_cloud_pub;
    ros::Publisher door_in_head_cloud_pub;
    ros::Publisher handle_pub;
double tx;
double ty;
double tz;
double qx;
double qy;
double qz;
double qw;
int a=0;


void move_to_frame(const PointCloudT::Ptr &input, const std::string &target_frame, PointCloudT::Ptr &output) {
	ROS_INFO("Transforming Input Point Cloud to %s frame...",  target_frame.c_str() );
	ROS_INFO("    Input Cloud Size: %zu", input->size());
	if (tf_listener->resolve(input->header.frame_id) == tf_listener->resolve(target_frame)) {
		output = input;
		return;
	}
	while (ros::ok()){
		tf::StampedTransform stamped_transform;
		try{
			// Look up transform
			tf_listener->lookupTransform(target_frame, input->header.frame_id, ros::Time(0), stamped_transform);

			// Apply transform
			pcl_ros::transformPointCloud(*input, *output, stamped_transform);
			std::cout << "X of one point is "<<output->points[50].x << std::endl;
			std::cout <<"Y of one point is "<< output->points[50].y << std::endl;
			std::cout << "Z of one point is "<<output->points[50].z << std::endl;

			// Store Header Details
			output->header.frame_id = target_frame;
			pcl_conversions::toPCL(ros::Time::now(), output->header.stamp);

			break;
		}
			//keep trying until we get the transform
		catch (tf::TransformException &ex){
			ROS_ERROR_THROTTLE(1, "%s", ex.what());
			ROS_WARN_THROTTLE(1,"    Waiting for transform from cloud frame (%s) to %s frame. Trying again", input->header.frame_id.c_str(), target_frame.c_str());
			continue;
		}
	}
}

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg){

handle_pcl->points.clear();
    sensor_msgs::PointCloud2 object_msg; // door
    sensor_msgs::PointCloud2 scene_msg; // GTX RGB-D picture of the scene  
 std::cout << "1111111111111111111111111111111"<< msg->header.frame_id<<std::endl;
 	// transfer scene from GTX frame to base_link frame
    pcl::fromROSMsg(*msg,*scene_pc); 
    door_in_head_cloud_pub.publish(scene_pc);
	move_to_frame(scene_pc,"base_link",scene_pc);
	pcl::toROSMsg(*scene_pc, scene_msg);

    pcl::toROSMsg(*stored_pc, object_msg);
    srv.request.object_cloud = object_msg;
    srv.request.target_cloud = scene_msg;


    ROS_INFO("Calling service");
    if (object_localize_client.call(srv)){
        ROS_INFO("Service returned");

        std::cout << "Position (x,y,z):" << srv.response.pose_offset.position.x << " "
                                         << srv.response.pose_offset.position.y << " "
                                         << srv.response.pose_offset.position.z << " "
                                         << std::endl;
        std::cout << "Quat (x,y,z,w):" << srv.response.pose_offset.orientation.x << " "
                                       << srv.response.pose_offset.orientation.y << " "
                                       << srv.response.pose_offset.orientation.z << " "
                                       << srv.response.pose_offset.orientation.w 
                                       << std::endl;
    tx = srv.response.pose_offset.position.x;    
    ty = srv.response.pose_offset.position.y;
    tz = srv.response.pose_offset.position.z;
    qx = srv.response.pose_offset.orientation.x;
    qy = srv.response.pose_offset.orientation.y;
    qz = srv.response.pose_offset.orientation.z;
    qw = srv.response.pose_offset.orientation.w;
    Vector3f pose_translation(tx,  ty, tz);

        RotMat3f pose_rotation;                                                
        pose_rotation = quat_to_R(srv.response.pose_offset.orientation);       

        ROS_INFO ("    | %6.3f %6.3f %6.3f | ", pose_rotation (0,0), pose_rotation (0,1), pose_rotation (0,2));
        ROS_INFO ("R = | %6.3f %6.3f %6.3f | ", pose_rotation (1,0), pose_rotation (1,1), pose_rotation (1,2));
        ROS_INFO ("    | %6.3f %6.3f %6.3f | ", pose_rotation (2,0), pose_rotation (2,1), pose_rotation (2,2));
        ROS_INFO ("t = < %0.3f, %0.3f, %0.3f >\n", pose_translation (0), pose_translation (1), pose_translation (2));

        Eigen::Matrix4f SE3_transform;
        SE3_transform.block<3,3>(0,0) = pose_rotation;
        SE3_transform.block<3,1>(0,3) = pose_translation;        
        SE3_transform (3,0) = 0.0;  SE3_transform (3,1) = 0.0; SE3_transform (3,2) = 0.0; SE3_transform (3,3) = 1.0;         

        ROS_INFO ("    | %f %f %f | ", SE3_transform (0,0), SE3_transform (0,1), SE3_transform (0,2));
        ROS_INFO ("R = | %f %f %f | ", SE3_transform (1,0), SE3_transform (1,1), SE3_transform (1,2));
        ROS_INFO ("    | %f %f %f | ", SE3_transform (2,0), SE3_transform (2,1), SE3_transform (2,2));
        ROS_INFO ("t = < %0.3f, %0.3f, %0.3f >", SE3_transform (0,3), SE3_transform (1,3), SE3_transform (2,3));
        ROS_INFO ("Last Row: < %0.7f, %0.7f, %0.7f,%0.7f  >", SE3_transform (3,0), SE3_transform (3,1), SE3_transform (3,2), SE3_transform (3,3) );


        PointCloudT::Ptr object_transformed (new PointCloudT);    
        pcl::transformPointCloud (*stored_pc, *object_transformed, SE3_transform);
/*		move_to_frame(object_transformed, "map", object_transformed);
*/
        pcl::toROSMsg(*object_transformed, object_transformed_cloud_msg);
        object_trans_cloud_pub.publish(object_transformed_cloud_msg);

	// handle position 
        pcl::PointXYZ handle;
        handle.x = srv.response.pose_offset.position.x;
        handle.y = srv.response.pose_offset.position.y;
        handle.z = srv.response.pose_offset.position.z;
        handle_pcl->points.push_back(handle);

	}
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "look_for_door");

    std::string package_path = ros::package::getPath("object_registration");
    std::string savepath = package_path + "/pcd_saved_files/";
    std::string load_filename="kleenexdoor.pcd";
    pcl::io::loadPCDFile(savepath + load_filename, *stored_pc);///////////////////////////////////////////////////////////////////////////////////////////////////////
    stored_pc->header.frame_id = "base_link";

	ros::NodeHandle nh;
	tf::TransformBroadcaster br;
	tf::Transform transform;
	stored_pc->header.stamp = ros::Time::now().toNSec() / 1000ull; // Convert from ns to us

	ros::Subscriber registered_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 10, cloud_callback);
    object_localize_client = nh.serviceClient<door_recognition::ObjectLocalize>("object_registration/object_localizer_service");

	tf_listener = new tf::TransformListener;

    object_trans_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("object_trans", 0);    
    door_in_baselink_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("door_in_baselink", 0);    
    door_in_head_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("door_in_head", 0);    
    handle_pub = nh.advertise<sensor_msgs::PointCloud2>("handle_point", 0);    
    ros::Rate r(20);
    object_transformed_cloud_msg.header.frame_id = "base_link"; 

    while(true){
    transform.setOrigin( tf::Vector3(tx, ty, tz) );
    transform.setRotation( tf::Quaternion(qx, qy, qz, qw) );
    if (handle_pcl->points.size() > 0)
    {
		try{
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "door"));

		}
		catch (tf::TransformException &ex){
			ROS_ERROR_THROTTLE(1, "%s", ex.what());
			continue;
		}
        handle_pub.publish(handle_pcl);
    }
  		door_in_baselink_cloud_pub.publish(stored_pc);
        ros::spinOnce();
        r.sleep();
    }
}


