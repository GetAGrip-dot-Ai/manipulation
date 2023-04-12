#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double tau = 2 * M_PI;
static const std::string PLANNING_GROUP = "arm";
namespace rvt = rviz_visual_tools;

class Commander
{

  public:
    
    Commander(){

    }

    Commander(int argc, char** argv)
    {
      ros::init(argc, argv, "commander_node");
      ros::NodeHandle node_handle;
      ros::AsyncSpinner spinner(1);
      std::cout<<"hello my name is "<<std::endl;
      spinner.start();
    }

    void moveToPose(geometry_msgs::Pose target_pose)
    {
      moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      moveit_visual_tools::MoveItVisualTools visual_tools("base");
      visual_tools.deleteAllMarkers();

      // Remote control is an introspection tool that allows users to step through a high level script
      // via buttons and keyboard shortcuts in RViz
      visual_tools.loadRemoteControl();

      // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
      Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
      text_pose.translation().z() = 1.0;
      visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

      // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
      visual_tools.trigger();

      // Getting Basic Information
      // ^^^^^^^^^^^^^^^^^^^^^^^^^
      // We can print the name of the reference frame for this robot
      ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

      // We can also print the name of the end-effector link for this group.
      ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

      // We can get a list of all the groups in the robot:
      ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
      std::copy(move_group_interface.getJointModelGroupNames().begin(),
                move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

      // move_group_interface.setPlanningTime(15.0);
      // set pose
      tf2::Quaternion q;
      q.setRPY(M_PI/2,0,0); //kinova
      geometry_msgs::Quaternion quat;
      quat = tf2::toMsg(q);
      geometry_msgs::Pose constrained_pose;
      constrained_pose.orientation = quat;
      constrained_pose.position = target_pose.position;

      // plan
      move_group_interface.setPoseTarget(constrained_pose);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      // // visualize plan
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
      visual_tools.publishAxisLabeled(target_pose, "pose1");
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
      visual_tools.prompt("execute?");

      // execute plan
      move_group_interface.move();
    }

    void moveToPreGraspPose(geometry_msgs::Pose target_pose)
    {
      moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      moveit_visual_tools::MoveItVisualTools visual_tools("base");
      visual_tools.deleteAllMarkers();

      // Remote control is an introspection tool that allows users to step through a high level script
      // via buttons and keyboard shortcuts in RViz
      visual_tools.loadRemoteControl();

      // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
      Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
      text_pose.translation().z() = 1.0;
      visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

      // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
      visual_tools.trigger();

      // Getting Basic Information
      // ^^^^^^^^^^^^^^^^^^^^^^^^^
      // We can print the name of the reference frame for this robot
      ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

      // We can also print the name of the end-effector link for this group.
      ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

      // We can get a list of all the groups in the robot:
      ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
      std::copy(move_group_interface.getJointModelGroupNames().begin(),
                move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

      // move_group_interface.setPlanningTime(15.0);
      // set pose
      tf2::Quaternion q;
      q.setRPY(M_PI/2,0,0); //kinova
      geometry_msgs::Quaternion quat;
      quat = tf2::toMsg(q);
      geometry_msgs::Pose constrained_pose;
      constrained_pose.orientation = quat;
      constrained_pose.position.x = target_pose.position.x;
      constrained_pose.position.y = target_pose.position.y + 0.1;
      constrained_pose.position.z = target_pose.position.z;

      // plan
      move_group_interface.setPoseTarget(constrained_pose);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      // // visualize plan
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
      visual_tools.publishAxisLabeled(target_pose, "pose1");
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
      visual_tools.prompt("execute?");

      // execute plan
      move_group_interface.move();
    }


    void followCartesianPath()
    {
      moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      moveit_visual_tools::MoveItVisualTools visual_tools("base");
      visual_tools.deleteAllMarkers();

      // Remote control is an introspection tool that allows users to step through a high level script
      // via buttons and keyboard shortcuts in RViz
      visual_tools.loadRemoteControl();

      // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
      Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
      text_pose.translation().z() = 1.0;
      visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

      // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
      visual_tools.trigger();

      std::copy(move_group_interface.getJointModelGroupNames().begin(),
                move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

      std::vector<geometry_msgs::Pose> waypoints;
      geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
      geometry_msgs::Pose new_pose = current_pose.pose;
      new_pose.position.y -= 0.1;
      waypoints.push_back(new_pose);  // move out by 10cm

      // We want the Cartesian path to be interpolated at a resolution of 1 cm
      // which is why we will specify 0.01 as the max step in Cartesian
      // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
      // Warning - disabling the jump threshold while operating real hardware can cause
      // large unpredictable motions of redundant joints and could be a safety issue
      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

      // Visualize the plan in RViz
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
      for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
      visual_tools.trigger();
      visual_tools.prompt("Cartesian Move?");
      // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
      // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
      // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
      // Pull requests are welcome.
      //
      // execute traj
      move_group_interface.execute(trajectory);

    }
};


// int main(int argc, char** argv)
// {

//   Commander comm;
//   geometry_msgs::Pose target_pose;
//   target_pose.position.x = 0.4;
//   target_pose.position.y = -0.3;
//   target_pose.position.z = 0.4;
//   comm.moveToPose(target_pose);
//   comm.followCartesianPath();
// }



