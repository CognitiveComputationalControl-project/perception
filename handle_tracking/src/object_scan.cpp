#include "object_scan.h"


Handle_manager::~Handle_manager(){}
Handle_manager::Handle_manager()
{
    x_left= 1000;
    
}

void Handle_manager::Publish_visualized_marker_first(const geometry_msgs::PoseStamped Pose)
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

    double temp_dist=0.05;

    //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
    visual_marker.scale.x = std::abs(temp_dist);
    visual_marker.scale.y = std::abs(temp_dist);
    visual_marker.scale.z = std::abs(temp_dist);

    visual_marker.color.r = 0.7;
    visual_marker.color.g = 0.4;
    visual_marker.color.b = 0.2;
    visual_marker.color.a = 0.85;

    handlemiddletarget_pub.publish(visual_marker);


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

void Handle_manager::grasp_point_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_INFO("Recieved grasp point");
    // ///////// marker for handle
    // visualization_msgs::Marker visual_marker; 
    // visual_marker.header.frame_id = "base_link"; 
    // visual_marker.header.stamp = ros::Time::now();
    // visual_marker.id = 0;
    // uint32_t shape = visualization_msgs::Marker::SPHERE;
    // visual_marker.type = shape;

    // visual_marker.pose.position.x = msg->pose.position.x;
    // visual_marker.pose.position.y = msg->pose.position.y;
    // visual_marker.pose.position.z = msg->pose.position.z;

    // visual_marker.pose.orientation.x = 0.0;
    // visual_marker.pose.orientation.y = 0.0;
    // visual_marker.pose.orientation.z = 0.0;
    // visual_marker.pose.orientation.w = 1.0;

    // double temp_dist=0.5;

    // //ROS_INFO("temp dist : %.3lf, temp dist2 : %.3lf, temp dist3 : %.3lf",temp_dist,temp_dist2,temp_dist3);
    // visual_marker.scale.x = std::abs(temp_dist);
    // visual_marker.scale.y = std::abs(temp_dist);
    // visual_marker.scale.z = std::abs(temp_dist);

    // visual_marker.color.r = 0.0;
    // visual_marker.color.g = 0.7;
    // visual_marker.color.b = 0.2;
    // visual_marker.color.a = 0.85;

    // handletarget_pub.publish(visual_marker);
}

void Handle_manager::marker_array_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{

for (int i = 0 ; i < msg->markers.size(); i++)
    {
    grasp_pose.header.stamp = ros::Time::now();
    grasp_pose.header.frame_id = "head_rgbd_sensor_rgb_frame";

      if ( (x_left > msg->markers[i].pose.position.x) /*&& abs(msg->markers[i].pose.position.y)< 0.2*/)
        { 
          // td::cout << "assignment";
        x_left = msg->markers[i].pose.position.x;
        grasp_pose.pose=msg->markers[i].pose;
        grasp_pose.header = msg->markers[i].header;

        }
    }
    
    Publish_visualized_marker_first(grasp_pose);

    //transform
    tf::StampedTransform transform_sensor_base;
    listener.waitForTransform("head_rgbd_sensor_rgb_frame","odom",  ros::Time(0), ros::Duration(2.0));
    try{ 
    listener.lookupTransform("head_rgbd_sensor_rgb_frame","odom", ros::Time(0), transform_sensor_base);
    listener.transformPose (std::string("odom"), grasp_pose,  grasp_transformed_pose) ; 

    }   
    catch (tf::TransformException &ex){
    }
    
    //std::cout<<grasp_transformed_pose.header.frame_id<<std::endl;
    ROS_INFO("_x : %.3lf, _y : %.3lf, _z : %.3lf \n ", grasp_transformed_pose.pose.position.x, grasp_transformed_pose.pose.position.y, grasp_transformed_pose.pose.position.z) ; 
    grasp_transformed_pose.header.stamp=ros::Time::now();
    grasp_transformed_pose.header.frame_id="odom";
    Publish_visualized_marker(grasp_transformed_pose);
    grasp_pub.publish(grasp_transformed_pose);

}

void Handle_manager::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    global_pose[0]=msg->pose.position.x;
    global_pose[1]=msg->pose.position.y;


   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

    global_pose[2]=yaw_tf;


   tf::StampedTransform Camera_transform;
   listener.waitForTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time(0), ros::Duration(5.0));
   listener.lookupTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time(0), Camera_transform);
   double yaw_angle_camera =   tf::getYaw(Camera_transform.getRotation()); 

   // Camera_angle=yaw_angle_camera;


}


void Handle_manager::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

    Head_Pos[0]=msg->position[9];           //pan
    Head_Pos[1]=msg->position[10];          //tilt
    
    // ROS_INFO("Head moving : %s",IsHeadMoving);
    // 0.000138, -3.8e-05,
    // 0.000138, -0.001567,

}



void Handle_manager::Human_MarkerarrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{

}

void Handle_manager::keyboard_callback(const keyboard::Key::ConstPtr& msg)
{

    ROS_INFO("Received Keyboard");
    std::cout<<msg->code<<std::endl;
    if(msg->code==116)      //if keyboard input is "t"
    {
        
    }
}

void Handle_manager::getCameraregion()
{

    // double global_robot_x= global_pose[0];
    // double global_robot_y= global_pose[1];
    // double global_robot_theta = global_pose[2]+Head_Pos[0];


    // // std::cout<<"theta:"<<Robot_Pos[2]<<",head :"<<Head_Pos[0]<<", total :"<<global_robot_theta<<std::endl;


    // double m_1=tan(30*3.141592/180);
    // double m_2=tan(30*3.141592/180);

    // visiblie_idx_set.clear();

    // global_robot_theta=0.0;
    // //Iteration for belief grid
    // for(int i(0);i<static_belief_map.info.width;i++)
    //  for(int j(0);j<static_belief_map.info.height;j++)
    // {
    //  int belief_map_idx=j*static_belief_map.info.height+i;

    //  // double map_ogirin_x = static_belief_map.info.origin.position.x+global_robot_x;
    //  // double map_ogirin_y = static_belief_map.info.origin.position.y+global_robot_y;

    //  double map_ogirin_x = static_belief_map.info.origin.position.x;
    //  double map_ogirin_y = static_belief_map.info.origin.position.y;


    //  double trans_vector_x=(i+0.5)*static_belief_map.info.resolution;
    //  double trans_vector_y=(j+0.5)*static_belief_map.info.resolution;

    //  double rot_trans_vector_x = cos(global_robot_theta)*trans_vector_x-sin(global_robot_theta)*trans_vector_y;
    //  double rot_trans_vector_y = sin(global_robot_theta)*trans_vector_x+cos(global_robot_theta)*trans_vector_y;

    //  double belief_global_x=map_ogirin_x+rot_trans_vector_x;
    //  double belief_global_y=map_ogirin_y+rot_trans_vector_y;


    //  //solve
    //  bool line1_result =getlinevalue(1,belief_global_x,belief_global_y);
    //  bool line2_result =getlinevalue(2,belief_global_x,belief_global_y);


    //  if( line1_result && line2_result )
    //  {
    //      static_belief_map.data[belief_map_idx]=30;  
    //      visiblie_idx_set.push_back(belief_map_idx);                 //save cell_id 
    //  }
    //  else
    //      static_belief_map.data[belief_map_idx]=0.0; 
    // }



}


