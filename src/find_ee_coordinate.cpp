/* Seven Dof arm planning code */

/* Author : Lentin Joseph */
 

//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char **argv)

{

	ros::init(argc, argv, "seven_dof_arm_planner");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);



	spinner.start();

	moveit::planning_interface::MoveGroupInterface group("arm");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	
	geometry_msgs::PoseStamped robot_pose;
	robot_pose = group.getCurrentPose();

	geometry_msgs::Pose current_position;
	current_position = robot_pose.pose;

	/*Retrive position and orientation */
	geometry_msgs::Point exact_pose = current_position.position;
	geometry_msgs::Quaternion exact_orientation = current_position.orientation;

	std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
	std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<<std::endl;

	



	ROS_INFO("Reference frame : %s",group.getPlanningFrame().c_str());

	ROS_INFO("Reference frame : %s",group.getEndEffectorLink().c_str());
	
	
	//sleep(4.0);

ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
moveit_msgs::AttachedCollisionObject attached_object;
attached_object.link_name = "grasping_frame_joint";
/* The header must contain a valid TF frame*/
attached_object.object.header.frame_id = "grasping_frame";
/* The id of the object */
attached_object.object.id = "box";

/* A default pose */
geometry_msgs::Pose pose;
pose.orientation.w = 1.0;

/* Define a box to be attached */
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.01;
primitive.dimensions[1] = 0.01;
primitive.dimensions[2] = 0.1;

attached_object.object.primitives.push_back(primitive);
attached_object.object.primitive_poses.push_back(pose);

attached_object.object.operation = attached_object.object.ADD;

ROS_INFO("Adding the object into the world at the location of the right wrist.");
moveit_msgs::PlanningScene planning_scene;
planning_scene.world.collision_objects.push_back(attached_object.object);
planning_scene.is_diff = true;
planning_scene_diff_publisher.publish(planning_scene);


sleep(2);



//sleep_time.sleep();
/*



geometry_msgs::Pose target_pose1;

target_pose1.orientation.x = 0;
target_pose1.orientation.y = 0;
target_pose1.orientation.z = 0;
target_pose1.orientation.w = 1;
target_pose1.position.y = 0.0;
target_pose1.position.x = 0.28;
target_pose1.position.z = 0.35;
group.setPoseTarget(target_pose1);
group.move();

target_pose1.orientation.x = 0;
target_pose1.orientation.y = 0;
target_pose1.orientation.z = 0;
target_pose1.orientation.w = 1;
target_pose1.position.y = 0.0;
target_pose1.position.x = 0.34;
target_pose1.position.z = 0.35;
group.setPoseTarget(target_pose1);
group.move();

target_pose1.orientation.x = 0;
target_pose1.orientation.y = 0;
target_pose1.orientation.z = 0;
target_pose1.orientation.w = 1;
target_pose1.position.y = 0.0;
target_pose1.position.x = 0.34;
target_pose1.position.z = 0.4;
group.setPoseTarget(target_pose1);
group.move();

target_pose1.orientation.x = 0;
target_pose1.orientation.y = 0;
target_pose1.orientation.z = 0;
target_pose1.orientation.w = 1;
target_pose1.position.y = 0.0;
target_pose1.position.x = 0.28;
target_pose1.position.z = 0.4;
group.setPoseTarget(target_pose1);
group.move();

*/
	ros::shutdown();
//	return 0;




}
