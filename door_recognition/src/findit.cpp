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

#include "quat_helper.h"
#include "geometry_msgs/Pose.h"
typedef pcl::PointXYZRGB PointNT;

typedef pcl::PointCloud<PointNT> PointCloudT;
sensor_msgs::PointCloud2 object_transformed_cloud_msg;    

ros::ServiceClient door_detector_client;
door_recognition::ObjectLocalize srv; 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  stored_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
ros::ServiceClient object_localize_client;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg){

    sensor_msgs::PointCloud2 object_msg;
    sensor_msgs::PointCloud2 scene_msg;    

    pcl::toROSMsg(*stored_pc, object_msg);
    scene_msg = *msg;
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

        Vector3f pose_translation(srv.response.pose_offset.position.x,  srv.response.pose_offset.position.y,  srv.response.pose_offset.position.z);
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
        pcl::toROSMsg(*object_transformed, object_transformed_cloud_msg);
        object_transformed_cloud_msg.header.frame_id = "map"; 
        // convert position to translation
        // convert quaternion to 3x3 rotation matrix

        // construct eigen matrix 4x4 again
        // transform pointcloud to visualize


        // create tf::Stamped Transform 
        // transform.setRotation()
        // transform.setOrigin()
}

}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "look_for_door");

    std::string package_path = ros::package::getPath("object_registration");
    std::string savepath = package_path + "/pcd_saved_files/";
    std::string load_filename="door.pcd";
    pcl::io::loadPCDFile(savepath + load_filename, *stored_pc);///////////////////////////////////////////////////////////////////////////////////////////////////////
    stored_pc->header.frame_id = "/map";

	ros::NodeHandle nh;
	    stored_pc->header.stamp = ros::Time::now().toNSec() / 1000ull; // Convert from ns to us

	ros::Subscriber registered_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 10, cloud_callback);
    object_localize_client = nh.serviceClient<door_recognition::ObjectLocalize>("object_registration/object_localizer_service");

    ros::Publisher object_trans_cloud_pub;

    object_trans_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("object_trans", 0);    
    ros::Rate r(20);

    while(true){
 
        object_trans_cloud_pub.publish(object_transformed_cloud_msg);
        ros::spinOnce();
        r.sleep();
    }
}


