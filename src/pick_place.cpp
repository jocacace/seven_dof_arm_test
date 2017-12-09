
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

	//Waiting for scene initialization
	sleep(2);


	//--- objects into the scene
	moveit::planning_interface::PlanningSceneInterface current_scene;
	moveit_msgs::CollisionObject grasping_object;
	grasping_object.id = "grasping_object";

	//---Add grasping object to the scene
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.03;
	primitive.dimensions[1] = 0.03;
	primitive.dimensions[2] = 0.08;

	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position.y =  0.0;
	pose.position.x =  0.33;
	pose.position.z =  0.35;

	grasping_object.primitives.push_back(primitive);
	grasping_object.primitive_poses.push_back(pose);
	grasping_object.operation = grasping_object.ADD;
	grasping_object.header.frame_id = "base_link";
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(grasping_object);
	//---


	// Once all of the objects (in this case just one) have been added to the
	// vector, we tell the planning scene to add our new box
	current_scene.addCollisionObjects(collision_objects);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	const robot_state::JointModelGroup *joint_model_group =
  group.getCurrentState()->getJointModelGroup("arm");

	

	//---approaching
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.x = 0;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	target_pose1.orientation.w = 1;
	target_pose1.position.y = 0.0;
	target_pose1.position.x = 0.32;
	target_pose1.position.z = 0.35;
	group.setPoseTarget(target_pose1);
	group.move();

	sleep(2);

	/*
	moveit::core::RobotStatePtr current_state = group.getCurrentState();
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	joint_group_positions[joint_group_positions.size()-1] = 3.14;  // radians
	group.setJointValueTarget(joint_group_positions);

	bool success = group.plan(my_plan);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

	group.move();



	geometry_msgs::PoseStamped robot_pose;
	robot_pose = group.getCurrentPose();

	geometry_msgs::Pose current_position;
	current_position = robot_pose.pose;

	geometry_msgs::Point exact_pose = current_position.position;
	geometry_msgs::Quaternion exact_orientation = current_position.orientation;

	std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
	std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<<std::endl;

	*/

	//---grasping
	target_pose1.orientation.x = 0;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	target_pose1.orientation.w = 1;
	target_pose1.position.y = 0.0;
	target_pose1.position.x = 0.37;
	target_pose1.position.z = 0.35;
	group.setPoseTarget(target_pose1);
	group.move();
	
	//---attach object to the robot
	moveit_msgs::AttachedCollisionObject attacched_object;
	attacched_object.link_name = "grasping_frame";
	attacched_object.object = grasping_object;
	current_scene.applyAttachedCollisionObject( attacched_object );
	sleep(2);

	//--move far
	target_pose1.orientation.x = 0;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	target_pose1.orientation.w = 1;
	target_pose1.position.y = 0.0;
	target_pose1.position.x = 0.34;
	target_pose1.position.z = 0.4;
	group.setPoseTarget(target_pose1);
	group.move();
	sleep(2);



	target_pose1.orientation.x = -1;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	target_pose1.orientation.w = 0;
	target_pose1.position.y = -0.1;
	target_pose1.position.x = 0.34;
	target_pose1.position.z = 0.4;
	group.setPoseTarget(target_pose1);
	group.move();
	//---

	target_pose1.orientation.x = -1;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	target_pose1.orientation.w = 0;
	target_pose1.position.y = -0.1;
	target_pose1.position.x = 0.34;
	target_pose1.position.z = 0.35;
	group.setPoseTarget(target_pose1);
	group.move();


	grasping_object.operation = grasping_object.REMOVE;
	attacched_object.link_name = "grasping_frame";
	attacched_object.object = grasping_object;
	current_scene.applyAttachedCollisionObject( attacched_object );
	

	target_pose1.orientation.x = -1;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	target_pose1.orientation.w = 0;
	target_pose1.position.y = -0.1;
	target_pose1.position.x = 0.32;
	target_pose1.position.z = 0.35;
	group.setPoseTarget(target_pose1);
	group.move();



	
	


	ros::shutdown();

}




/*

//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char **argv) {

	ros::init(argc, argv, "seven_dof_arm_planner");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//initialize move group and planning scene
	moveit::planning_interface::MoveGroupInterface group("arm");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//---put objects into the scene
	moveit::planning_interface::PlanningSceneInterface current_scene;
	moveit_msgs::CollisionObject grasping_object;
	grasping_object.id = "grasping_object";

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.03;
	primitive.dimensions[1] = 0.03;
	primitive.dimensions[2] = 0.08;

	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position.y =  0.0;
	pose.position.x =  0.33;
	pose.position.z =  0.35;

	grasping_object.primitives.push_back(primitive);
	grasping_object.primitive_poses.push_back(pose);
	grasping_object.operation = grasping_object.ADD;
	grasping_object.header.frame_id = "base_link";
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(grasping_object);

	// Once all of the objects (in this case just one) have been added to the
	// vector, we tell the planning scene to add our new box
	current_scene.addCollisionObjects(collision_objects);

	//approaching pose
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.x = 0;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	target_pose1.orientation.w = 1;
	target_pose1.position.y = 0.0;
	target_pose1.position.x = 0.32;
	target_pose1.position.z = 0.35;
	group.setPoseTarget(target_pose1);
	group.move();

	sleep(2);

	//grasping
	target_pose1.position.y = 0.0;
	target_pose1.position.x = 0.37;
	target_pose1.position.z = 0.35;
	group.setPoseTarget(target_pose1);
	group.move();


	sleep(2);

	moveit_msgs::AttachedCollisionObject attacched_object;
	attacched_object.link_name = "grasping_frame";
	attacched_object.object = grasping_object;
	current_scene.applyAttachedCollisionObject( attacched_object );


	//detaching
	target_pose1.position.y = 0.0;
	target_pose1.position.x = 0.37;
	target_pose1.position.z = 0.4;
	group.setPoseTarget(target_pose1);
	group.move();


	sleep(2);

	target_pose1.position.y = 0.0;
	target_pose1.position.x = 0.28;
	target_pose1.position.z = 0.4;
	group.setPoseTarget(target_pose1);
	group.move();


	ros::shutdown();

}
*/
