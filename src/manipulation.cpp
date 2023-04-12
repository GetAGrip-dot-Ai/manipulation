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
#include <std_msgs/Int8.h>
#include <manipulation/Obstacle.h>
#include <manipulation/harvest.h>

const double tau = 2 * M_PI;
static const std::string PLANNING_GROUP = "arm";
namespace rvt = rviz_visual_tools;

// fake ee world
int done=0;
int done2=0;
int done3=0;

std::vector<geometry_msgs::Pose> obstacle_poses;
std::vector<shape_msgs::SolidPrimitive> obstacle_primitives;
geometry_msgs::Pose poi_pose;
geometry_msgs::Pose basket_pose; // need to hardcode in srv callback
geometry_msgs::Pose reset_pose; // need to hardcode in srv callback
geometry_msgs::Pose approach_pose; // need to hardcode in srv callback
int approach_pose_num = 0;


// manipulation::multi_frame mf_srv;
// ros::ServiceClient mf_client;

// fake ee world
ros::Publisher ee_pub;

// remove obstacles from scene
void removeObstacles(std::vector<std::string> object_ids){
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


// move to target pose
void moveToPose(geometry_msgs::Pose target_pose)
{
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
  constrained_pose.position.x -= (0.192 - 0.061525); // offset between our end-effector gripping point and tool_frame for robotiq ee
  move_group_interface.setPoseTarget(constrained_pose);

  // make plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // // visualize plan
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // execute plan
  // visual_tools.prompt("execute?");
  move_group_interface.move();
}


// update POI 
void updatePOICallback(const geometry_msgs::Pose msg)
{
  // real svd world 
	poi_pose = msg;

  // fake svd world
  // if (done==0){
  //   done = 1;
  //   ROS_INFO_STREAM("POI Pose");
  //   ROS_INFO_STREAM(poi_pose);
  //   ROS_INFO_STREAM("Moving to pre-grap POI");
  //   poi_pose.position.x -= 0.1;
  //   moveToPose(poi_pose);

  //   ROS_INFO_STREAM("EE finished opening, moving to POI");
  //   poi_pose.position.x += 0.1;
  //   moveToPose(poi_pose);

    // ROS_INFO_STREAM("Publishing to EE World");
    // std_msgs::Int8 state;
    //   state.data = 8;
    // ee_pub.publish(state);
  // }
}


// ee response callback (for the fake world)
void eeResponseCallback(const std_msgs::Int8 msg)
{
	// int response = msg.data;
  // if (done2==0){
  //   done2 = 1;
  //   if(response==1){
  //     ROS_INFO_STREAM("EE finished opening, moving to POI");
  //     poi_pose.position.x += 0.1;
  //     moveToPose(poi_pose);
  //     std_msgs::Int8 state;
  //     state.data = 10;
  //     ee_pub.publish(state);
  //   }
  //   else{
  //     ROS_INFO_STREAM("Waiting for EE to open");
  //   }
  // }
  // if (done2==1 && done3==0){
  //   done3=1;
  //   if(response==1){
  //     ROS_INFO_STREAM("EE finished extracting, moving to basket pose");
  //     reset_pose.position.x = 0;
  //     reset_pose.position.y = -0.4;
  //     reset_pose.position.z = 0.5;
  //     moveToPose(reset_pose);
  //   }
  //   else{
  //     ROS_INFO_STREAM("Waiting for EE to extract");
  //   }
  // }
}


// update obstacle callback
void updateObstaclesCallback(const manipulation::Obstacle msg)
{
  obstacle_poses = msg.pose;
  obstacle_primitives = msg.primitive;

//   if (done2==0){
//     done2 = 1;
//     int n = obstacle_poses.size();
//     int m = obstacle_primitives.size();
//     std::cout << "n:"<<n <<"m:"<<m<<std::endl;


//     addObstacle(obstacle_poses[0], obstacle_primitives[0], std::to_string(0));
//     addObstacle(obstacle_poses[1], obstacle_primitives[1], std::to_string(1));
//     addObstacle(obstacle_poses[2], obstacle_primitives[2], std::to_string(2));

//     // for(int i=0; i<2; i++){
//     //   addObstacle(obstacle_poses[i], obstacle_primitives[i], std::to_string(i));
//     // }
//   }
}


// harvest service callback
bool harvestSrvCallback(manipulation::harvest::Request& request, manipulation::harvest::Response& response)
{
  int state = request.req_id;
  switch (state) {
    
    case 0:{ // move to reset pose
    std::cout << "moving to reset pose" << std::endl;
    try {
      reset_pose.position.x = -0.1;
      reset_pose.position.y = -0.2;
      reset_pose.position.z = 0.5;
      moveToPose(reset_pose);
      response.reply = 1;
    }
    catch (...) {
      std::cout << "move to reset pose failed" << std::endl;
      response.reply = 0;
    }
    return 1;
  }

    case 1:{ // approach plant positions
      std::cout << "approaching plant" << std::endl;
      if (approach_pose_num == 0){
        approach_pose.position.x = -0.1;
        approach_pose.position.y = -0.4;
        approach_pose.position.z = 0.3;
        moveToPose(approach_pose);
        approach_pose_num = 1;
      }
      else if (approach_pose_num == 1){
        approach_pose.position.x = -0.1;
        approach_pose.position.y = -0.1;
        approach_pose.position.z = 0.4;
        moveToPose(approach_pose);
        approach_pose_num = 2;
      }
      else if (approach_pose_num == 2){
        approach_pose.position.x = -0.1;
        approach_pose.position.y = 0.2;
        approach_pose.position.z = 0.5;
        moveToPose(approach_pose);
        approach_pose_num = 3;
      }
      else {
        response.reply = 0;
      }
      response.reply = 1;
      return 1;
    }

    // case 2:{ // multiframe   //ishu: chance the move may fail
    //   std::cout << "running multiframe" << std::endl;
    //   int frame_id = 0;
    //   while(frame_id<5){

    //     mf_srv.request.req_id = 0;
    //     if(mf_client.call(mf_srv)){
    //         ROS_INFO("Received Response");
    //         int did_yolo = harvest_srv.response.reply;
    //         // ROS_INFO_STREAM(success);
    //         if(did_yolo==1){
    //             ROS_INFO("Manipulation: Done with Yolo, moving to new waypoint");
    //             // this->next_state = State::APPROACH_PLANT_POSITIONS;
    //             // increment waypoint index -- make function that takes current pose & index
    //         }
    //         if(did_yolo==1){
    //             ROS_INFO("Manipulation: Done with Yolo, moving to new waypoint");
    //             // this->next_state = State::APPROACH_PLANT_POSITIONS;
    //             // increment waypoint index -- make function that takes current pose & index
    //         }
    //     }
    //     else{
    //         ROS_ERROR("Manipulation: No reponse received from perception multiframe server");
    //         response.reply = 0;
    //     }
    //   }

    //   ROS_INFO("Manipulation: Multi_frame state completed");
    //   response.reply = 1

    // }

    case 3:{ // create pepper obstacles and move to pre-grasp POI pose

      // create obstacles
      int m = obstacle_poses.size();
      for(int i=0; i<m; i++){
        try {
          addObstacle(obstacle_poses[i], obstacle_primitives[i], std::to_string(i));
        }
        catch (...) {
          std::cout << "removing obstacles failed" << std::endl;
          response.reply = 0;
        }
      }

      // move to pre-grasp POI pose
      std::cout << "moving to pre-grasp pose" << std::endl;
      try {
        geometry_msgs::Pose pregrasp_pose = poi_pose;
        pregrasp_pose.position.y += 0.1;
        moveToPose(pregrasp_pose);
      }
      catch (...) {
        std::cout << "move to pre-grasp POI pose failed" << std::endl;
        response.reply = 0;
      }

      response.reply = 1;
      return 1;
    }
    
    case 5:{ // move to POI
      std::cout << "moving to POI" << std::endl;
      try {
        moveToPose(poi_pose);
      }
      catch (...) {
        std::cout << "move to POI pose failed" << std::endl;
        response.reply = 0;
      }
      response.reply = 1;
      return 1;
    }

    case 7:{ // move to basket drop pose and remove all obstacles
      std::cout << "moving to basket" << std::endl;
      try {
        basket_pose.position.x = -0.1;
        basket_pose.position.y = -0.2;
        basket_pose.position.z = 0.5;
        moveToPose(basket_pose);
      }
      catch (...) {
        std::cout << "move to basket pose failed" << std::endl;
        response.reply = 0;
      }

      // remove all existing obstacles in the scene
      std::cout << "removing all pepper obstacles" << std::endl;
      int n = obstacle_poses.size();
      std::vector<std::string> object_ids;
      for(int i=0; i<n; i++){
        object_ids.push_back(std::to_string(i));
      }
      try {
        removeObstacles(object_ids);
      }
      catch (...) {
        std::cout << "adding obstacle failed" << std::endl;
        response.reply = 0;
      }

      response.reply = 1;
      return 1;

    }
  
  }
}


int main(int argc, char **argv) {

    // start node
    ros::init(argc, argv, "manipulation");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // multiframe srv
    // mf_client = n.serviceClient<manipulation::multi_frame>("/perception/multi_frame");

    // subscribers
    ros::Subscriber poi_sub = n.subscribe("/perception/peduncle/poi", 1000, updatePOICallback);
    ros::Subscriber obstacle_sub = n.subscribe("/perception/pepper/bbox", 1000, updateObstaclesCallback);

    // fake ee stuff
    ros::Subscriber ee_sub = n.subscribe("/end_effector/harvest_rsp", 1000, eeResponseCallback);
    // ee_pub = n.advertise<std_msgs::Int8>("/end_effector/harvest_req", 1000);

    // harvest srv
    ros::ServiceServer harvest_server = n.advertiseService("/manipulation/harvest",harvestSrvCallback);

    
    ros::Rate loop_rate(1);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}

