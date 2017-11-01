#include "object_scan.h"


Handle_manager::~Handle_manager(){}
Handle_manager::Handle_manager()
{
	
	
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

	Head_Pos[0]=msg->position[9];			//pan
	Head_Pos[1]=msg->position[10];			//tilt
	
	// ROS_INFO("Head moving : %s",IsHeadMoving);
	// 0.000138, -3.8e-05,
	// 0.000138, -0.001567,

}



void Handle_manager::Human_MarkerarrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{

	// if(IsHeadMoving)
	// {
	// 	std::cout<<"head is moving so return function"<<std::endl;
	// 		return;
	// }
	
	// //Human marker from yolo detection
	// human_occupied_idx.clear();
	// Cur_detected_human.clear();
	

	// num_of_detected_human=msg->markers.size();

	// if(num_of_detected_human>0)
	// 	Cur_detected_human.resize(num_of_detected_human);
	// else
	// {
	// 	return;

	// }
	// // ROS_INFO("number of detected human : %d",num_of_detected_human);
	
	
	// for(int i(0);i<num_of_detected_human;i++)
	// {
	// 	geometry_msgs::Vector3Stamped gV, tV;

	//     gV.vector.x = msg->markers[i].pose.position.x;
	//     gV.vector.y = msg->markers[i].pose.position.y;
	//     gV.vector.z = msg->markers[i].pose.position.z;

	//     // std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
	//     tf::StampedTransform maptransform;
 //   		listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(5.0));
 //   		listener.lookupTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), maptransform);
	    
	//     gV.header.stamp = ros::Time();
	//     gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
	//     listener.transformVector(std::string("/map"), gV, tV);
	    	    
	// 	Cur_detected_human[i].resize(2,0.0);
	// 	// Cur_detected_human[i][0]=msg->markers[i].pose.position.x;;
	// 	// Cur_detected_human[i][1]=msg->markers[i].pose.position.y;
	// 	//MKMKMKMK 0624
	// 	Cur_detected_human[i][0]=tV.vector.x+global_pose[0];
	// 	Cur_detected_human[i][1]=tV.vector.y+global_pose[1];
	// 	int human_mapidx=CoordinateTransform_Global2_beliefMap(Cur_detected_human[i][0],Cur_detected_human[i][1]);
	// 	human_occupied_idx.push_back(human_mapidx);
	// 	// std::cout<<"human index : "<<human_mapidx<<std::endl;
	// 	//ROS_INFO("Human idx : %d, Received position x : %.3lf, y : %.3lf",i, msg->markers[i].pose.position.x,msg->markers[i].pose.position.y);
	// }

	// put_human_occ_map_yolo();
	

}

void Handle_manager::keyboard_callback(const keyboard::Key::ConstPtr& msg)
{

	ROS_INFO("Received Keyboard");
	std::cout<<msg->code<<std::endl;
	if(msg->code==116)		//if keyboard input is "t"
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
	// 	for(int j(0);j<static_belief_map.info.height;j++)
	// {
	// 	int belief_map_idx=j*static_belief_map.info.height+i;

	// 	// double map_ogirin_x = static_belief_map.info.origin.position.x+global_robot_x;
	// 	// double map_ogirin_y = static_belief_map.info.origin.position.y+global_robot_y;

	// 	double map_ogirin_x = static_belief_map.info.origin.position.x;
	// 	double map_ogirin_y = static_belief_map.info.origin.position.y;


	// 	double trans_vector_x=(i+0.5)*static_belief_map.info.resolution;
	// 	double trans_vector_y=(j+0.5)*static_belief_map.info.resolution;

	// 	double rot_trans_vector_x = cos(global_robot_theta)*trans_vector_x-sin(global_robot_theta)*trans_vector_y;
	// 	double rot_trans_vector_y = sin(global_robot_theta)*trans_vector_x+cos(global_robot_theta)*trans_vector_y;

	// 	double belief_global_x=map_ogirin_x+rot_trans_vector_x;
	// 	double belief_global_y=map_ogirin_y+rot_trans_vector_y;


	// 	//solve
	// 	bool line1_result =getlinevalue(1,belief_global_x,belief_global_y);
	// 	bool line2_result =getlinevalue(2,belief_global_x,belief_global_y);


	// 	if( line1_result && line2_result )
	// 	{
	// 		static_belief_map.data[belief_map_idx]=30;	
	// 		visiblie_idx_set.push_back(belief_map_idx);					//save cell_id 
	// 	}
	// 	else
	// 		static_belief_map.data[belief_map_idx]=0.0;	
	// }



}


