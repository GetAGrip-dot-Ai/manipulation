#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <std_msgs/Int16.h>
#include <manipulation/Obstacle.h>
// #include <manipulation/dpoi.h>
#include <manipulation/visual_servo.h>
#include <manipulation/harvest.h>
#include <manipulation/multi_frame.h>



// Defining your own ROS stream color

namespace pc
{
  enum PRINT_COLOR
  {
    BLACK,
    RED,
    GREEN,
    YELLOW,
    BLUE,
    MAGENTA,
    CYAN,
    WHITE,
    ENDCOLOR
  };

  std::ostream& operator<<(std::ostream& os, PRINT_COLOR c)
  {
    switch(c)
    {
      case BLACK    : os << "\033[1;30m"; break;
      case RED      : os << "\033[1;31m"; break;
      case GREEN    : os << "\033[1;32m"; break;
      case YELLOW   : os << "\033[1;33m"; break;
      case BLUE     : os << "\033[1;34m"; break;
      case MAGENTA  : os << "\033[1;35m"; break;
      case CYAN     : os << "\033[1;36m"; break;
      case WHITE    : os << "\033[1;37m"; break;
      case ENDCOLOR : os << "\033[0m";    break;
      default       : os << "\033[1;37m";
    }
    return os;
  }
} //namespace pc

#define ROS_BLACK_STREAM(x)   ROS_INFO_STREAM(pc::BLACK   << x << pc::ENDCOLOR)
#define ROS_RED_STREAM(x)     ROS_INFO_STREAM(pc::RED     << x << pc::ENDCOLOR)
#define ROS_GREEN_STREAM(x)   ROS_INFO_STREAM(pc::GREEN   << x << pc::ENDCOLOR)
#define ROS_YELLOW_STREAM(x)  ROS_INFO_STREAM(pc::YELLOW  << x << pc::ENDCOLOR)
#define ROS_BLUE_STREAM(x)    ROS_INFO_STREAM(pc::BLUE    << x << pc::ENDCOLOR)
#define ROS_MAGENTA_STREAM(x) ROS_INFO_STREAM(pc::MAGENTA << x << pc::ENDCOLOR)
#define ROS_CYAN_STREAM(x)    ROS_INFO_STREAM(pc::CYAN    << x << pc::ENDCOLOR)

#define ROS_BLACK_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, pc::BLACK   << x << pc::ENDCOLOR)
#define ROS_RED_STREAM_COND(c, x)     ROS_INFO_STREAM_COND(c, pc::RED     << x << pc::ENDCOLOR)
#define ROS_GREEN_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, pc::GREEN   << x << pc::ENDCOLOR)
#define ROS_YELLOW_STREAM_COND(c, x)  ROS_INFO_STREAM_COND(c, pc::YELLOW  << x << pc::ENDCOLOR)
#define ROS_BLUE_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, pc::BLUE    << x << pc::ENDCOLOR)
#define ROS_MAGENTA_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, pc::MAGENTA << x << pc::ENDCOLOR)
#define ROS_CYAN_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, pc::CYAN    << x << pc::ENDCOLOR)


const double tau = 2 * M_PI;
static const std::string PLANNING_GROUP = "arm";
namespace rvt = rviz_visual_tools;


std::vector<geometry_msgs::Pose> obstacle_poses;
std::vector<shape_msgs::SolidPrimitive> obstacle_primitives;
geometry_msgs::Pose poi_pose;
// geometry_msgs::Pose updated_poi_pose;
geometry_msgs::Pose basket_pose; 
geometry_msgs::Pose reset_pose; 
geometry_msgs::Pose approach_pose;
geometry_msgs::Pose pregrasp_pose;
geometry_msgs::Pose pregrasp_pose2;
float dx;
float dy;
float dz;
int approach_pose_num = 0;
int in_case_10 = 0;
bool depth_failed = 0;

float pregrasp_offset = 0.075; // may need to change tuned number in move to pregrasp in service
float pregrasp_offset_far = 0.215;
float pregrasp_offset_z = 0.03; 

// initialize sub-services with perception
manipulation::multi_frame mf_srv;
ros::ServiceClient mf_client;
manipulation::visual_servo vs_srv;
ros::ServiceClient vs_client;


// remove obstacles from scene
void removeObstacles(std::vector<std::string> object_ids){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.loadRemoteControl();
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  planning_scene_interface.removeCollisionObjects(object_ids);
  visual_tools.trigger();
}

// add obstacles to the scene
void addObstacle(geometry_msgs::Pose obstacle_pose, shape_msgs::SolidPrimitive primitive, std::string obstacle_id){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // define a collision object ROS message for the robot to avoid
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();

  // The id of the object is used to identify it
  collision_object.id = obstacle_id;

  // add the collision object into the world
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(obstacle_pose);
  collision_object.operation = collision_object.ADD;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  // visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'n' in the RvizVisualToolsGui window to once the collision object appears in RViz");
}

// update obstacle callback
void updateObstaclesCallback(const manipulation::Obstacle msg)
{
  obstacle_poses = msg.pose;
  obstacle_primitives = msg.primitive;
}

// move to target pose
bool moveToPose(geometry_msgs::Pose target_pose){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // print the name of the reference frame for this robot
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  //  print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  // list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // we call the planner to compute the plan and visualize it.
  // move_group_interface.setPlanningTime(15.0);
  tf2::Quaternion q;
  q.setRPY(-M_PI/2,-M_PI/2,-M_PI/2);; //kinova
  geometry_msgs::Quaternion quat;
  quat = tf2::toMsg(q);
  geometry_msgs::Pose constrained_pose;
  constrained_pose.orientation = quat;
  constrained_pose.position = target_pose.position;
  constrained_pose.position.x -= 0.03; // (0.192 - 0.061525); // offset between our end-effector gripping point and tool_frame for robotiq ee
  move_group_interface.setPoseTarget(constrained_pose);

  // make plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  if(!success){
    return false;
  }

  // // visualize plan
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // return success;
  // execute plan
  // visual_tools.prompt("execute?");

  moveit::planning_interface::MoveItErrorCode moved = move_group_interface.move();
  if (moved == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    return true;
  }
  return false;

}

// cartesian move to POI
bool cartMoveToPoiForVS(){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // move_group_interface.setMaxVelocityScalingFactor(0.5);
  // move_group_interface.setMaxAccelerationScalingFactor(0.1);

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
  geometry_msgs::Pose next_pose = current_pose.pose;

  // next_pose.position.z += dz;
  // waypoints.push_back(next_pose);

  // next_pose.position.y += dy;
  // waypoints.push_back(next_pose);

  next_pose.position.x += pregrasp_offset;
  waypoints.push_back(next_pose);

  depth_failed = 0;
  
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if(fraction<0.9){
    ROS_MAGENTA_STREAM("failed plan (fraction)");
    return false;
  }

  if(trajectory.joint_trajectory.points.empty()){
    ROS_MAGENTA_STREAM("the plan is cartesian yay");
  }
  else{
    ROS_MAGENTA_STREAM("the plan is maybe not cartesian -- i dont think this catch thing is right lol");
    // return false;
  }

  // sleep(15.0);
  // Set the execution duration of the trajectory
  const double duration = 20;
  double traj_duration = trajectory.joint_trajectory.points.back().time_from_start.toSec();
  double scale = duration / traj_duration;
  for (auto& point : trajectory.joint_trajectory.points) {
    point.time_from_start *= scale;
  }

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path to POI!!", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  // visual_tools.prompt("Execute multiframe move?");


  // You can execute a trajectory like this.
  
  moveit::planning_interface::MoveItErrorCode moved = move_group_interface.execute(trajectory);
  if (moved == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    return true;
  }
  return false;

  
}

// cartesian move to the poi position
bool cartMoveToPoi(){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // move_group_interface.setMaxVelocityScalingFactor(0.5);
  // move_group_interface.setMaxAccelerationScalingFactor(0.1);

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
  geometry_msgs::Pose next_pose = current_pose.pose;

  next_pose.position.x += pregrasp_offset;
  waypoints.push_back(next_pose);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // ROS_INFO_STREAM("DONE");
  if (fraction < 0.9){
    return false;
  }

  if(trajectory.joint_trajectory.points.empty()){
    ROS_MAGENTA_STREAM("the plan is cartesian yay");
  }
  else{
    ROS_MAGENTA_STREAM("the plan is maybe not cartesian -- i dont think this catch thing is right lol");
    // return false;
  }

  // sleep(15.0);
  // Set the execution duration of the trajectory
  const double duration = 20;
  double traj_duration = trajectory.joint_trajectory.points.back().time_from_start.toSec();
  double scale = duration / traj_duration;
  for (auto& point : trajectory.joint_trajectory.points) {
    point.time_from_start *= scale;
  }

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  // visual_tools.prompt("Execute multiframe move?");

  // You can execute a trajectory like this.
  moveit::planning_interface::MoveItErrorCode moved = move_group_interface.execute(trajectory);
  if (moved == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    return true;
  }
  return false;

}

// move down in Z to basket drop
bool moveDownToDrop(){
  ROS_CYAN_STREAM("moving down now");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // move_group_interface.setMaxVelocityScalingFactor(0.5);
  // move_group_interface.setMaxAccelerationScalingFactor(0.1);

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
  geometry_msgs::Pose next_pose = current_pose.pose;

  next_pose.position.z -= 0.03;
  waypoints.push_back(next_pose);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction < 0.9){
    return false;
  }

  // sleep(15.0);
  // Set the execution duration of the trajectory
  const double duration = 10;
  double traj_duration = trajectory.joint_trajectory.points.back().time_from_start.toSec();
  double scale = duration / traj_duration;
  for (auto& point : trajectory.joint_trajectory.points) {
    point.time_from_start *= scale;
  }

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  // visual_tools.prompt("Execute multiframe move?");

  // You can execute a trajectory like this.

  moveit::planning_interface::MoveItErrorCode moved = move_group_interface.execute(trajectory);
  if (moved == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    return true;
  }
  return false;
  

}

// cartesian move back to pregrasp pose
bool cartMoveToPreGrasp(){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // move_group_interface.setMaxVelocityScalingFactor(0.5);
  // move_group_interface.setMaxAccelerationScalingFactor(0.1);

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
  geometry_msgs::Pose next_pose = current_pose.pose;

  next_pose.position.x -= 0.05;
  waypoints.push_back(next_pose);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction < 0.9){
    return false;
  }

  // sleep(15.0);
  // Set the execution duration of the trajectory
  const double duration = 12;
  double traj_duration = trajectory.joint_trajectory.points.back().time_from_start.toSec();
  double scale = duration / traj_duration;
  for (auto& point : trajectory.joint_trajectory.points) {
    point.time_from_start *= scale;
  }

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  // visual_tools.prompt("Execute multiframe move?");

  // You can execute a trajectory like this.

  moveit::planning_interface::MoveItErrorCode moved = move_group_interface.execute(trajectory);
  if (moved == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    return true;
  }
  return false;
  
}

// move to basket drop pose
bool moveToBasket(){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("moving to joint state");

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // current pose
  geometry_msgs::Pose current_pose = move_group_interface.getCurrentPose().pose;

  // current state
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  double J0 = joint_group_positions[0];

  // find alpha (offset from base)
  float alpha = tan(current_pose.position.y / current_pose.position.x);
  if(alpha < M_PI/2){
    alpha -= 2*M_PI;
  }

  // set new J0 value
  joint_group_positions[0] = J0+alpha+M_PI/3; // move 60 degrees
  move_group_interface.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // if(!success){
  //   return false;
  // }

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  // visual_tools.prompt("execute?");
  // try{
  move_group_interface.move();
  // }
  // catch(...){
  //   ROS_INFO_STREAM("Move to 60 degrees failed");
  //   return false;
  // }
  // moveit::planning_interface::MoveItErrorCode moved = move_group_interface.move();
  // if (moved == moveit::planning_interface::MoveItErrorCode::SUCCESS){
  //   // return true;
  // }
  // // return false;

  // ros::NodeHandle node_handle;
  // ros::AsyncSpinner spinner(1);

  // cartesian z
  // bool moved1 = moveDownToDrop();
  // bool moved2 = moveDownToDrop();
  // bool moved3 = moveDownToDrop();
  // bool moved4 = moveDownToDrop();

  // if (moved1 && moved2 && moved3 && moved4){
  //   return true;
  // }
  // ROS_INFO_STREAM("moving down to basket failed");
  // return false;


//   std::vector<geometry_msgs::Pose> waypoints;
//   geometry_msgs::PoseStamped current_pose2 = move_group_interface.getCurrentPose();
//   geometry_msgs::Pose next_pose = current_pose2.pose;

//   next_pose.position.z -= 0.03;
//   waypoints.push_back(next_pose);

//   next_pose.position.z -= 0.03;
//   waypoints.push_back(next_pose);

//   next_pose.position.z -= 0.03;
//   waypoints.push_back(next_pose);

//   next_pose.position.z -= 0.03;
//   waypoints.push_back(next_pose);

//   moveit_msgs::RobotTrajectory trajectory;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;
//   double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

// // if (fraction < 0.9){
// //   moveToPose(next_pose);
// // }
// // else{
//   // sleep(15.0);
//   // Set the execution duration of the trajectory
//   const double duration = 40;
//   double traj_duration = trajectory.joint_trajectory.points.back().time_from_start.toSec();
//   double scale = duration / traj_duration;
//   for (auto& point : trajectory.joint_trajectory.points) {
//     point.time_from_start *= scale;
//   }
//   // Visualize the plan in RViz
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//   for (std::size_t i = 0; i < waypoints.size(); ++i)
//     visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
//   visual_tools.trigger();
//   // visual_tools.prompt("Execute multiframe move?");

  
//   moveit::planning_interface::MoveItErrorCode moved2 = move_group_interface.execute(trajectory);
//   if (moved2 == moveit::planning_interface::MoveItErrorCode::SUCCESS){
//     return true;
//   }
//   ROS_INFO_STREAM("moving down to basket failed");
//   return false;
          
  // }

  // MOVE 60 DEGREES
  // current state
  // moveit::core::RobotStatePtr current_state2 = move_group_interface.getCurrentState();
  // std::vector<double> joint_group_positions2;
  // current_state2->copyJointGroupPositions(joint_model_group, joint_group_positions2);
  // // set new J0 value
  // joint_group_positions2[0] = M_PI/3; // move to 0 degrees
  // move_group_interface.setJointValueTarget(joint_group_positions2);
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  // bool success2 = (move_group_interface.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);
  // // Visualize the plan in RViz
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // // visual_tools.prompt("execute?");
  // move_group_interface.move();

}

// multiframe moves
void multiframe(int frame_id){
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // get current pose
  geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();

  // set orientation
  tf2::Quaternion q;
  q.setRPY(-M_PI/2,-M_PI/2,-M_PI/2); //kinova
  geometry_msgs::Quaternion quat;
  quat = tf2::toMsg(q);

  geometry_msgs::Pose pose;
  pose.orientation = quat;
  pose.position = current_pose.pose.position;

  // ROS_INFO_STREAM("in frame_id "<< frame_id);
  ROS_CYAN_STREAM("in multiframe for frame_id "<< frame_id);

  // pose 1
  if (frame_id == 0){
    pose.position.z += 0.03;
  }
  // pose 2
  else if (frame_id==1){
    pose.position.y -= 0.03;
  }
  // pose 3
  else if (frame_id==2){
    pose.position.z -= 0.04;
  }
  // pose 4
  else if (frame_id==3){
    pose.position.y += 0.04;
  }
  // pose 5
  else{
    pose.position.z += 0.04;
  }

  move_group_interface.setPoseTarget(pose);
  // move_group_interface.setPlanningTime(15.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success1 = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // if(!success1){
  //   return false;
  // }

  // moveit::planning_interface::MoveItErrorCode moved = move_group_interface.move();
  // if (moved == moveit::planning_interface::MoveItErrorCode::SUCCESS){
  //   return true;
  // }
  // return false;

  move_group_interface.move();

}

// multiframe cartesian
bool cartMultiframe(int frame_id){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // move_group_interface.setMaxVelocityScalingFactor(0.5);
  // move_group_interface.setMaxAccelerationScalingFactor(0.1);

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
  geometry_msgs::Pose next_pose = current_pose.pose;

    // pose 1
  if (frame_id == 0){
    next_pose.position.z += 0.03;
    waypoints.push_back(next_pose);
  }
  // pose 2
  else if (frame_id==1){
    next_pose.position.y += 0.03;
    waypoints.push_back(next_pose);
  }
  // pose 3
  else if (frame_id==2){
    next_pose.position.y -= 0.03;
    waypoints.push_back(next_pose);
  }
  // pose 4
  else if (frame_id==3){
    next_pose.position.z += 0.03;
    waypoints.push_back(next_pose);
  }
  // pose 5
  else{
    next_pose.position.z += 0.03;
    waypoints.push_back(next_pose);
  }

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if(fraction < 0.9){
    return false;
  }

  // sleep(15.0);
  // Set the execution duration of the trajectory
  const double duration = 15;
  double traj_duration = trajectory.joint_trajectory.points.back().time_from_start.toSec();
  double scale = duration / traj_duration;
  for (auto& point : trajectory.joint_trajectory.points) {
    point.time_from_start *= scale;
  }

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  // visual_tools.prompt("Execute multiframe move?");

  // You can execute a trajectory like this.
  // move_group_interface.execute(trajectory);

  moveit::planning_interface::MoveItErrorCode moved = move_group_interface.execute(trajectory);
  if (moved == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    return true;
  }
  return false;

}

bool moveToPG2ForVS(){

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // print the name of the reference frame for this robot
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  //  print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  // list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // we call the planner to compute the plan and visualize it.
  // move_group_interface.setPlanningTime(15.0);
  tf2::Quaternion q;
  q.setRPY(-M_PI/2,-M_PI/2,-M_PI/2);; //kinova
  geometry_msgs::Quaternion quat;
  quat = tf2::toMsg(q);
  geometry_msgs::Pose constrained_pose;
  constrained_pose.orientation = quat;
  // constrained_pose.position = target_pose.position;
  // constrained_pose.position.x -= 0.03; // (0.192 - 0.061525); // offset between our end-effector gripping point and tool_frame for robotiq ee
  geometry_msgs::Pose current_pose = move_group_interface.getCurrentPose().pose;
  constrained_pose.position.y =  current_pose.position.y + dy;
  constrained_pose.position.z  = current_pose.position.z + dz;
  constrained_pose.position.x = current_pose.position.x + dx - 0.04; //UPDATE MAYBE!
  move_group_interface.setPoseTarget(constrained_pose);

  // make plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  if(!success){
    return false;
  }

  // // visualize plan
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(target_pose, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // return success;
  // execute plan
  // visual_tools.prompt("execute?");

  moveit::planning_interface::MoveItErrorCode moved = move_group_interface.move();
  if (moved == moveit::planning_interface::MoveItErrorCode::SUCCESS){
    return true;
  }
  return false;

}

// update POI
void updatePOICallback(const geometry_msgs::Pose msg)
{
  ROS_INFO_STREAM("in poi callback");
	poi_pose = msg;
}

// update POI from visual servoing
void deltaPOICallback(const geometry_msgs::Pose msg){
  // ROS_CYAN_STREAM("AAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
  dx = msg.position.x - 0.03;
  dy = msg.position.y;
  dz = msg.position.z;
  // ROS_BLUE_STREAM("dx: "<< dx <<" dy: "<< dy<<" dz: "<< dz);
}

// harvest service callback
bool harvestSrvCallback(manipulation::harvest::Request& request, manipulation::harvest::Response& response)
{
  int state = request.req_id;
  ROS_RED_STREAM("STATE: "<<state);

  // int state = 2;
  switch (state) {
    
    case 0:{ // move to reset pose
    std::cout << "moving to reset pose" << std::endl;
    reset_pose.position.x = 0.1;
    reset_pose.position.y = 0;
    reset_pose.position.z = 0.5;
    bool success = moveToPose(reset_pose);
    if(success==true){
      response.reply = 1;
    }
    else{
      response.reply = 0;
    }
    return 1;

  }

    case 1:{ // approach plant positions
      std::cout << "approaching plant" << std::endl;
      std::cout << "going to approach position "<< approach_pose_num << std::endl;
      bool moved;
      if (approach_pose_num == 0){
        approach_pose.position.x = 0.15;
        approach_pose.position.y = 0;
        approach_pose.position.z = 0.65;
        moved = moveToPose(approach_pose);
        // approach_pose_num = 1;
      }
      else if (approach_pose_num == 1){
        approach_pose.position.x = 0.1;
        approach_pose.position.y = 0.16;
        approach_pose.position.z = 0.6;
        moved = moveToPose(approach_pose);
        // approach_pose_num = 2;
      }
      else if (approach_pose_num == 2){
        approach_pose.position.x = 0.1;
        approach_pose.position.y = -0.16;
        approach_pose.position.z = 0.6;
        moved = moveToPose(approach_pose);
        // approach_pose_num = 3;
      }
      else if (approach_pose_num == 3){
        approach_pose.position.x = 0.1;
        approach_pose.position.y = 0;
        approach_pose.position.z = 0.53;
        moved = moveToPose(approach_pose);
        // approach_pose_num = 3;
      }
      else {
        response.reply = 0;
        return 1;
      }
      
      // if(moved){
      //   response.reply = 1;
      // }
      // else{
      //   approach_pose.position.x = 0.15;
      //   approach_pose.position.y = 0;
      //   approach_pose.position.z = 0.6;
      //   moveToPose(approach_pose);
      // }

      response.reply = 1;
      return 1;
    }

    case 2:{ // multiframe 

        ROS_INFO("running multiframe");
        int frame_id = 0;

        mf_srv.request.req_id = 0;
        int case_2_count = 0;
        while(!mf_client.call(mf_srv)){
          if (case_2_count%500==0)
              ROS_INFO("WAITING");
          case_2_count += 1;
        }
        
        // frames 0 to 1
        while(frame_id<2){
          ROS_INFO("In while loop");
          ROS_MAGENTA_STREAM("frame id: "<<frame_id);
          mf_srv.request.req_id = 0;
          ROS_INFO_STREAM("response from perception "<<mf_client.call(mf_srv));
            ROS_INFO("Received Response");
            int did_yolo = mf_srv.response.reply;
            if(did_yolo==1){
                ROS_RED_STREAM("Done with YOLO, moving to new waypoint");
                bool cart_mf = cartMultiframe(frame_id); // execute multiframe move
                frame_id += 1;
            }
        }

        if (frame_id==2){
          mf_srv.request.req_id = 1; // req: 1
          if(mf_client.call(mf_srv)){
            ROS_RED_STREAM("Request to process multi frames");
            int found_pepper = mf_srv.response.reply;
            if(found_pepper==1){
              ROS_BLUE_STREAM("POI FOUND");
              response.reply = 1; // system level respond true
            }
            else{
              ROS_BLUE_STREAM("NO PEPPERS FOUND");
              approach_pose_num += 1;
              response.reply = 0; // system level respond false
            }
          }
      }

      return 1;

    }

    case 3:{ // create pepper obstacles and move to pre-grasp POI pose

      // create obstacles
      // int m = obstacle_poses.size();
      // for(int i=0; i<m; i++){
      //   ROS_INFO_STREAM("Obstacle Size: ");
      //   ROS_INFO_STREAM(m);
      //   try {
      //     addObstacle(obstacle_poses[i], obstacle_primitives[i], std::to_string(i));
      //   }
      //   catch (...) {
      //     std::cout << "creating obstacles failed" << std::endl;
      //     response.reply = 0;
      //   }
      // }

      // move to pre-grasp POI pose
      std::cout << "moving to pre-grasp pose" << std::endl;
      pregrasp_pose = poi_pose;
      if(poi_pose.position.x < 0.3){
        // response.reply = 0;
        // return 1;
        depth_failed = 1;
        ROS_RED_STREAM("DEPTH FAILED");
        pregrasp_pose.position.x = 0.3;

      }
      else{
        pregrasp_pose.position.x -= (pregrasp_offset_far-0.015);
      }
      pregrasp_pose.position.z -= pregrasp_offset_z;
      bool success = moveToPose(pregrasp_pose);
      std::cout << "moved to pre grasp" << std::endl;

      if (success){
        std::cout << "moved to pre grasp" << std::endl;
        response.reply = 1;
      }
      else{
        std::cout << "move to pre-grasp POI pose failed" << std::endl;
        response.reply = 0;
      }
      ROS_BLUE_STREAM(response.reply);
      return 1;
    }

    case 7:{ // move back to pre-grasp poi, move to basket drop pose and remove all obstacles
      // move back to pre grasp poi
      std::cout << "moving to pre-grasp" << std::endl;
      bool cart_move1 = cartMoveToPreGrasp();
      bool cart_move2 = cartMoveToPreGrasp();
      bool cart_move3 = cartMoveToPreGrasp();
      bool cart_move4 = cartMoveToPreGrasp();
      if (cart_move1 && cart_move2 && cart_move3 && cart_move4){
          std::cout << "moved to pre grasp" << std::endl;
      }
      else{
        std::cout << "move to pre grasp failed" << std::endl;
        response.reply = 0; 
      }

      // move to basket
      std::cout << "moving to basket" << std::endl;
      bool moved_to_basket = moveToBasket();
      if (moved_to_basket){
          std::cout << "moved to basket successfully" << std::endl;
      }
      else{
        std::cout << "move to basket pose failed" << std::endl;
        response.reply = 0; 
      }
      // moveToBasket();

      // remove all existing obstacles in the scene
      // std::cout << "removing all pepper obstacles" << std::endl;
      // int n = obstacle_poses.size();
      // std::vector<std::string> object_ids;
      // for(int i=0; i<n; i++){
      //   object_ids.push_back(std::to_string(i));
      // }
      // try {
      //   removeObstacles(object_ids);
      // }
      // catch (...) {
      //   std::cout << "adding obstacle failed" << std::endl;
      //   response.reply = 0;
      // }

      response.reply = 1;
      return 1;

    }
  
    case 10:{ // visual servoing
      ROS_INFO("start visual servoing");

        ROS_INFO("visual servoing");
        vs_srv.request.req_id = 0;
        
        // waiting for response
        int wait_count = 0;
        while(!vs_client.call(vs_srv)){
          if (wait_count % 500 == 0){
            ROS_INFO("WAITING for visual servo server");
          }
          wait_count+=1;
        }

        // received response
        ROS_INFO("Received Response");
        ROS_RED_STREAM("response from perception");

        // ros::AsyncSpinner spinner(1);
        // spinner.start();
        // ros::spinOnce();

        // check reply
        int did_vs = vs_srv.response.reply;
        ros::Duration(5).sleep();

        // if VS succeeded, move to updated poi
        if(did_vs==1){
            ROS_BLUE_STREAM("Done with visual servoing, moving to updated POI");
            ros::spinOnce();
            // move to pregrasp 2 from mf poi
            // pregrasp_pose2 = poi_pose;
            // pregrasp_pose2.position.x -= (pregrasp_offset-0.015);
            bool success1 = moveToPG2ForVS();
            if (!success1){
              ROS_BLUE_STREAM("Moving to pre grasp 2 failed - telling system VS failed");
              response.reply = 0;
            }

            // cartesian move to upated poi 
            bool vs_moved_to_poi = cartMoveToPoiForVS();
            if(vs_moved_to_poi){
              ROS_BLUE_STREAM("Moved to updated POI");
              response.reply = 1;
            }
            else{
              ROS_BLUE_STREAM("Cartesian move to POI failed");
              response.reply = 0;
            }
        }
        // if VS failed, move to old poi
        else{
          ROS_BLUE_STREAM("Visual Servoing failed in perception world");
          // move to pre grasp 2 with joint space move
          pregrasp_pose2 = poi_pose;
          pregrasp_pose2.position.x -= (pregrasp_offset-0.015);
          pregrasp_pose2.position.z += (pregrasp_offset_z);
          bool success1 = moveToPose(pregrasp_pose2);
          if (!success1){
            ROS_BLUE_STREAM("Moving to pre grasp 2 failed - telling system VS failed");
            response.reply = 0;
          }
          
          // move to POI with cartesian move
          ROS_INFO_STREAM("Moving to POI");
          bool success2 = cartMoveToPoi();
          if (!success2){
            ROS_BLUE_STREAM("cartesian move to poi failed - telling system VS failed");
            response.reply = 0;
          }

          response.reply = 1; // tell system we moved to POI

        }

      // response.reply = 0;
      return 1;

    }
  }
}


int main(int argc, char **argv) {
  
  ROS_INFO("in main");

  // start node
  ros::init(argc, argv, "manipulation_node");
  ros::NodeHandle n;
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  // perception sercive clients
  mf_client = n.serviceClient<manipulation::multi_frame>("/perception/multi_frame");
  vs_client = n.serviceClient<manipulation::visual_servo>("/perception/visual_servo");

  // subscribers
  ros::Subscriber poi_sub = n.subscribe("/perception/peduncle/poi", 1000, updatePOICallback);
  ros::Subscriber delta_poi_sub = n.subscribe("/perception/peduncle/dpoi", 1000, deltaPOICallback);
  ros::Subscriber obstacle_sub = n.subscribe("/perception/pepper/bbox", 1000, updateObstaclesCallback);

  // harvest server service
  ros::ServiceServer harvest_server = n.advertiseService("/manipulation/harvest",harvestSrvCallback);
  
  ros::Rate loop_rate(1);
  int main_count=0;
  while(ros::ok()) {
    if (main_count%500==0)
      ROS_INFO("waiting for something to happen");
    main_count += 1;
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;

}

