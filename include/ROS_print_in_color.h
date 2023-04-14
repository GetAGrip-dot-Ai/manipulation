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
#include <manipulation/dpoi.h>
#include <manipulation/visual_servo.h>
#include <manipulation/harvest.h>
#include <manipulation/multi_frame.h>


// ^^^^^^^^^^ THINGS TO FIX ^^^^^^^^^^
// approach positions (and orientations)
// basket pose orientation
// camera TF
// throwing error if the move fails
// error handling: move to reset pose and try move again if error


const double tau = 2 * M_PI;
static const std::string PLANNING_GROUP = "arm";
namespace rvt = rviz_visual_tools;

// fake ee world -- can delete later
int done=0;
int done2=0;
int done3=0;
ros::Publisher ee_pub;

std::vector<geometry_msgs::Pose> obstacle_poses;
std::vector<shape_msgs::SolidPrimitive> obstacle_primitives;
geometry_msgs::Pose poi_pose;
geometry_msgs::Pose updated_poi_pose;
geometry_msgs::Pose basket_pose; 
geometry_msgs::Pose reset_pose; 
geometry_msgs::Pose approach_pose;
int approach_pose_num = 0;
int in_case_2 = 0;
int in_case_10 = 0;

// initialize sub-services with perception
manipulation::multi_frame mf_srv;
ros::ServiceClient mf_client;
manipulation::visual_servo vs_srv;
ros::ServiceClient vs_client;

// frames saved during multiframe moves
geometry_msgs::PoseStamped mf_frame0_pose;
geometry_msgs::PoseStamped mf_frame1_pose;
geometry_msgs::PoseStamped mf_frame2_pose;
geometry_msgs::PoseStamped mf_frame3_pose;
geometry_msgs::PoseStamped mf_frame4_pose;
geometry_msgs::PoseStamped mf_frame5_pose;


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

// move to target pose
void moveToPose(geometry_msgs::Pose target_pose){
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

// move to selected multi frame position
void moveToPrevFrame(int frame_id){
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
  // tf2::Quaternion q;
  // q.setRPY(-M_PI/2,-M_PI/2,-M_PI/2);; //kinova
  // geometry_msgs::Quaternion quat;
  // quat = tf2::toMsg(q);
  // geometry_msgs::Pose constrained_pose;
  // constrained_pose.orientation = quat;
  // constrained_pose.position = target_pose.position;
  // constrained_pose.position.x -= (0.192 - 0.061525); // offset between our end-effector gripping point and tool_frame for robotiq ee

  geometry_msgs::Pose frame_pose;

  // pose 1
  if (frame_id == 0){
    frame_pose = mf_frame0_pose.pose;
  }
  // pose 2
  else if (frame_id==1){
    frame_pose = mf_frame1_pose.pose;
  }
  // pose 3
  else if (frame_id==2){
    frame_pose = mf_frame2_pose.pose;
  }
  // pose 4
  else if (frame_id==3){
    frame_pose = mf_frame3_pose.pose;
  }
  // pose 5
  else{
    frame_pose = mf_frame4_pose.pose;
  }
  
  
  move_group_interface.setPoseTarget(frame_pose);

  // make plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // // visualize plan
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(target_pose, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // execute plan
  // visual_tools.prompt("execute?");
  move_group_interface.move();
}

// multiframe moves
void multiframe(int frame_id, bool first){
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

  ROS_INFO_STREAM("in frame_id "<< frame_id);

  if(first){
    mf_frame0_pose = move_group_interface.getCurrentPose();
  }

  // pose 1
  if (frame_id == 0){
    pose.position.z += 0.02;
    mf_frame1_pose = move_group_interface.getCurrentPose();
  }
  // pose 2
  else if (frame_id==1){
    pose.position.y -= 0.02;
    mf_frame2_pose = move_group_interface.getCurrentPose();
  }
  // pose 3
  else if (frame_id==2){
    pose.position.z -= 0.04;
    mf_frame3_pose = move_group_interface.getCurrentPose();
  }
  // pose 4
  else if (frame_id==3){
    pose.position.y += 0.04;
    mf_frame4_pose = move_group_interface.getCurrentPose();
  }
  // pose 5
  else{
    pose.position.z += 0.04;
    mf_frame5_pose = move_group_interface.getCurrentPose();
  }

  move_group_interface.setPoseTarget(pose);
  // move_group_interface.setPlanningTime(15.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success1 = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group_interface.move();

}

// update POI 
void updatePOICallback(const geometry_msgs::Pose msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO_STREAM("in poi callback");
  
  // real svd world 
	poi_pose = msg;

  // fake svd world
  if (done==0){
    done = 1;
    ROS_INFO_STREAM("POI Pose");
    ROS_INFO_STREAM(poi_pose);
    ROS_INFO_STREAM("Moving to pre-grap POI");
    poi_pose.position.x -= 0.1;
    moveToPose(poi_pose);

    ROS_INFO_STREAM("fake EE finished opening, moving to POI");
    poi_pose.position.x += 0.1;
    moveToPose(poi_pose);

    // ROS_INFO_STREAM("Publishing to EE World");
    // std_msgs::Int8 state;
    //   state.data = 8;
    // ee_pub.publish(state);
  }
}

// update POI from visual servoing
void deltaPOICallback(const manipulation::dpoi msg){
  float dy = msg.dy;
  float dz = msg.dz;
  updated_poi_pose = poi_pose;
  updated_poi_pose.position.y = poi_pose.position.y - dy;
  updated_poi_pose.position.z = poi_pose.position.z - dz;
}

// ee response callback (for the fake world) -- can delete later
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
  // int state = request.req_id;
  int state = 2;
  switch (state) {
    
    case 0:{ // move to reset pose
    std::cout << "moving to reset pose" << std::endl;
    try {
      reset_pose.position.x = 0.3;
      reset_pose.position.y = 0;
      reset_pose.position.z = 0.7;
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
        approach_pose.position.x = 0.5;
        approach_pose.position.y = 0.2;
        approach_pose.position.z = 0.5;
        moveToPose(approach_pose);
        approach_pose_num = 1;
      }
      else if (approach_pose_num == 1){
        approach_pose.position.x = 0.5;
        approach_pose.position.y = 0.2;
        approach_pose.position.z = 0.5;
        moveToPose(approach_pose);
        approach_pose_num = 2;
      }
      else if (approach_pose_num == 2){
        approach_pose.position.x = 0.5;
        approach_pose.position.y = -0.2;
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

    case 2:{ // multiframe 

      if (in_case_2==0){  // maybe should get rid of this if
        in_case_2 = 1;

        ROS_INFO("running multiframe");
        int frame_id = 0;

        mf_srv.request.req_id = 0; // added this today but didn't test
        while(!mf_client.call(mf_srv)){
          ROS_INFO("WAITING");
        }
        
        // frames 0 to 5
        while(frame_id<5){

          ROS_INFO("In while loop");
          ROS_INFO_STREAM(frame_id);

          mf_srv.request.req_id = 0;
          ROS_INFO_STREAM("response from perception "<<mf_client.call(mf_srv));
            ROS_INFO("Received Response");
            int did_yolo = mf_srv.response.reply;
            if(did_yolo==1){
                ROS_INFO("Manipulation: Done with Yolo, moving to new waypoint");
                if(frame_id==0){
                  multiframe(frame_id, true); // execute multiframe move
                }
                else{
                  multiframe(frame_id, false);
                }
                frame_id += 1;
            }
        }

        // frame 6
        if (frame_id==5){
          mf_srv.request.req_id = 1; // req: 1
          if(mf_client.call(mf_srv)){
            ROS_INFO("Request to process multi frames");
            int selected_frame = mf_srv.response.reply;
            if(selected_frame==-1){
              ROS_INFO("no pepper found, telling system we are done");
              response.reply = 0; // system level respond false
            }
            else{
              ROS_INFO("pepper found in frame: ");
              ROS_INFO_STREAM(selected_frame);
              ROS_INFO("move to new frame");
              // ROS_INFO("Multiframe Done - No more POIs exist");
              moveToPrevFrame(selected_frame);
              mf_srv.request.req_id = 2;
              if(mf_client.call(mf_srv)){
                ROS_INFO("Got a response that POI was published, tell system we are done with multiframe");
                response.reply = 1; // system level respond true
              }
            }
          }
        }

      }

      // in_case_2 = 0;  // added this today but didnt test
      return 1;

    }

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
        basket_pose.position.x = 0.4;
        basket_pose.position.y = 0.4;
        basket_pose.position.z = 0.2;
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
  
    case 10:{ // visual servoing

      if (in_case_10==0){
        in_case_10 = 1;

        ROS_INFO("visual servoing");
        vs_srv.request.req_id = 0;
        
        // waiting for response
        while(!vs_client.call(vs_srv)){
          ROS_INFO("WAITING");
        }

        // received response
        ROS_INFO("Received Response");
        ROS_INFO_STREAM("response from perception "<<vs_client.call(vs_srv));

        // check reply
        int did_vs = vs_srv.response.reply;
        // if VS succeeded, move to new POI
        if(did_vs==1){
            ROS_INFO("Done with visual servoing, moving to updated POI");
            moveToPose(updated_poi_pose); // need to add a check that this move was successful
            ROS_INFO("Moved to updated POI");
            response.reply = 1;
        }
        // if VS failed, tell system
        else{
          ROS_INFO("Visual Servoing failed");
          response.reply = 0; // tell system visual servoing failed
        }

      }

      in_case_10 = 0;
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

  // fake ee stuff
  // ros::Subscriber ee_sub = n.subscribe("/end_effector/harvest_rsp", 1000, eeResponseCallback);
  // ee_pub = n.advertise<std_msgs::Int8>("/end_effector/harvest_req", 1000);

  // harvest server service
  ros::ServiceServer harvest_server = n.advertiseService("/manipulation/harvest",harvestSrvCallback);
  
  ros::Rate loop_rate(1);
  while(ros::ok()) {
    ROS_INFO("wah");
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;

}

