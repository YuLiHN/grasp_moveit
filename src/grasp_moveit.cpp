#include <memory>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/clock.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <std_srvs/srv/empty.hpp>
#include <grasp_srv/srv/grasp_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <geometry_msgs/msg/transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include "tf2_ros/static_transform_broadcaster.h"



using std::placeholders::_1;
using std::placeholders::_2;

using franka_msgs::action::Grasp;
using franka_msgs::action::Move;
const std::string MOVE_GROUP = "panda_arm";



class GraspMoveit : public rclcpp::Node
{
  public:
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr exec_grasp_service, move_to_start_service, test_function_service;
  rclcpp::Client<grasp_srv::srv::GraspArray>::SharedPtr grasp_client;
  sensor_msgs::msg::JointState start_joint_state, rot_right, rot_left;

  geometry_msgs::msg::Pose example_target_pose;

  rclcpp_action::Client<Grasp>::SharedPtr gripper_grasp_action;
  rclcpp_action::Client<Move>::SharedPtr gripper_move_action;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;


  GraspMoveit(): Node("grasp_moveit",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)), 
                  move_group_interface(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
  { 
    // for the test
    example_target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = 0.0;
    msg.position.z = 0.5;
    return msg;
    }();

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped world2robot;
    world2robot.header.stamp = this->get_clock()->now();
    world2robot.header.frame_id = "world";
    world2robot.child_frame_id = "panda_link0";

    world2robot.transform.rotation.x=0;
    world2robot.transform.rotation.y=0;
    world2robot.transform.rotation.z=0;
    world2robot.transform.rotation.w=1;
    world2robot.transform.translation.x=0;
    world2robot.transform.translation.y=0;
    world2robot.transform.translation.z=0;

    geometry_msgs::msg::TransformStamped robot2marker;
    robot2marker.header.stamp = this->get_clock()->now();
    robot2marker.header.frame_id = "aruco_marker_frame";
    robot2marker.child_frame_id = "world";

    robot2marker.transform.rotation.x=0;
    robot2marker.transform.rotation.y=0;
    robot2marker.transform.rotation.z=0;
    robot2marker.transform.rotation.w=1;
    robot2marker.transform.translation.x=-0.302;
    robot2marker.transform.translation.y=0.097;
    robot2marker.transform.translation.z=0;

    

    tf_static_broadcaster_->sendTransform(robot2marker);
    tf_static_broadcaster_->sendTransform(world2robot);
    

    // create predefined states
    std::string name_prefix="panda_joint";
    std::vector<std::string> joints_name;
    for(int i=1;i<=7;i++)
    {
      joints_name.push_back(name_prefix+std::to_string(i));
      RCLCPP_INFO(this->get_logger(),name_prefix+std::to_string(i));
    }
    start_joint_state.name=joints_name;
    start_joint_state.position = std::vector<double,std::allocator<double>>{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};

    rot_right.name=joints_name;
    rot_right.position = std::vector<double,std::allocator<double>>{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, 2.25};

    rot_left.name =joints_name;
    rot_left.position = std::vector<double,std::allocator<double>>{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, -0.84};

    // initialize services and client
    exec_grasp_service=this->create_service<std_srvs::srv::Empty>("execute_grasp", std::bind(&GraspMoveit::execute_grasp_cb,this,_1,_2));
    move_to_start_service=this->create_service<std_srvs::srv::Empty>("move_to_start", std::bind(&GraspMoveit::move_to_start_cb,this,_1,_2));
    test_function_service=this->create_service<std_srvs::srv::Empty>("test", std::bind(&GraspMoveit::test_function_cb,this,_1,_2));
    grasp_client=this->create_client<grasp_srv::srv::GraspArray>("grasp_array");

    // initialize actions
    gripper_move_action=rclcpp_action::create_client<Move>(this,"/panda_gripper/move");
    gripper_grasp_action=rclcpp_action::create_client<Grasp>(this,"/panda_gripper/grasp");
    

    move_group_interface.startStateMonitor();
    

    RCLCPP_INFO(this->get_logger(), "initialization successful! waiting for the service /execute_grasp call");
  }

  bool ask_continue()
  {
    char ans;
    while (1)
    {
      RCLCPP_INFO(this->get_logger(),"Continue? (y/n)?");
      std::cin>>ans;
      if (ans=='y'|| ans=='Y') {return true;}
      else if(ans=='n' || ans=='N') {return false;}
    }
  }

  bool move_to_start()
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // move_group_interface.setStartState(*move_group_interface.getCurrentState());
    // move_group_interface.setStartStateToCurrentState();
    if (!move_group_interface.setJointValueTarget(start_joint_state)){return false;}
    auto success = move_group_interface.plan(plan);

    if(!success){return false;}
    if(!ask_continue()){return false;}
    move_group_interface.execute(plan);

    return true;

  }

  void cleanup()
  {
    auto conames = planning_scene_interface.getKnownObjectNames();
    planning_scene_interface.removeCollisionObjects(conames);
    
    // std::vector< moveit_msgs::msg::CollisionObject> empty_vec;
    // planning_scene_interface.applyCollisionObject(empty_vec);
    // the reason to use applyCollisionObject() rather than removeCollisionObjects() is the latter is asynchronous;
    // http://docs.ros.org/en/noetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1PlanningSceneInterface.html

    move_group_interface.clearPoseTarget();
    return;
  }

  bool move_to_pose(const geometry_msgs::msg::PoseStamped &target_pose)
  {
    
    geometry_msgs::msg::PoseStamped pre_grasp_pose = target_pose;
    pre_grasp_pose.pose.position.z-=0.05;

    move_group_interface.setPoseTarget(pre_grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan_pre_grasp;
    auto success_pre_grasp = move_group_interface.plan(plan_pre_grasp);

    // Execute the plan
    if (!success_pre_grasp){return false;}
    if(!ask_continue()){return false;}
    move_group_interface.execute(plan_pre_grasp);
    // rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(this->get_logger(),"Moved to pre grasp pose successfully!");

    // move_group_interface.setStartStateToCurrentState();
    move_group_interface.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan_target_pose;
    auto success_target_pose = move_group_interface.plan(plan_target_pose);

    if (!success_target_pose){return false;}
    if(!ask_continue()){return false;}
    move_group_interface.execute(plan_target_pose);
    // rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
  }

  bool close_gripper()
  {
    if (!this->gripper_grasp_action->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server /panda_gripper/grasp not available after waiting");
      return false;
    }

    auto goal = Grasp::Goal();
    goal.width=0.0;
    goal.speed=0.03;
    goal.force=3;
    goal.epsilon.inner=0.5;
    goal.epsilon.outer=0.5;
    gripper_grasp_action->async_send_goal(goal);

    return true;

  }

  bool open_gripper()
  {
    if (!this->gripper_move_action->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server /panda_gripper/move not available after waiting");
      return false;
    }

    auto goal = Move::Goal();
    goal.width = 0.08;
    goal.speed = 0.8;
    gripper_move_action->async_send_goal(goal);

    return true;

  }

  bool move_straight_up(const geometry_msgs::msg::PoseStamped &target_pose)
  {
    // move_group_interface.setStartStateToCurrentState();
    // auto higher_pose = move_group_interface.getCurrentPose().pose;
    auto higher_pose = target_pose;
    higher_pose.pose.position.z += 0.10; // First move up (z)

    move_group_interface.setPoseTarget(higher_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = move_group_interface.plan(plan);
    if(!success){return false;}
    if(!ask_continue()){return false;}
    move_group_interface.execute(plan);

    return true;
  }

  bool test_grasp()
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // turn right
    // move_group_interface.setStartStateToCurrentState();
    move_group_interface.setJointValueTarget(rot_right);
    auto success_right = move_group_interface.plan(plan);
    if(!success_right){return false;}
    if(!ask_continue()){return false;}
    move_group_interface.execute(plan);
    

    // turn left
    // move_group_interface.setStartStateToCurrentState();
    move_group_interface.setJointValueTarget(rot_left);
    auto success_left = move_group_interface.plan(plan);
    if(!success_left){return false;}
    if(!ask_continue()){return false;}
    move_group_interface.execute(plan);

    // go back to start state
    // move_group_interface.setStartStateToCurrentState();
    move_group_interface.setJointValueTarget(start_joint_state);
    auto success_start = move_group_interface.plan(plan);
    if(!success_start){return false;}
    if(!ask_continue()){return false;}
    move_group_interface.execute(plan);
    
    return true;
  }


  bool select_target_pose(const std::vector<geometry_msgs::msg::PoseStamped> &grasp_array, geometry_msgs::msg::PoseStamped &target_pose)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for(auto grasp_pose : grasp_array)
    {
      move_group_interface.setPoseTarget(grasp_pose);

      if (move_group_interface.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        target_pose=grasp_pose;
        return true;
      }
    }

    return false;
  }

  private:
  void move_to_start_cb(const std_srvs::srv::Empty::Request::SharedPtr request,
                      std_srvs::srv::Empty::Response::SharedPtr response)
  {
    move_to_start();
  }


  void execute_grasp_cb(const std_srvs::srv::Empty::Request::SharedPtr request,
                      std_srvs::srv::Empty::Response::SharedPtr response)
  {
    cleanup();

    if (!move_to_start()){cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"Moved to start successfully! Waiting for the grasps");
    
    while (!grasp_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "grasp service not available, waiting again...");
    }
    auto send_request = std::make_shared<grasp_srv::srv::GraspArray::Request>();
    auto future = grasp_client->async_send_request(send_request);
    future.wait(); // lead to deadlock?

    // add collision obejcts
    std::vector<geometry_msgs::msg::PoseStamped> grasp_array_result = future.get()->grasp_array;

    // grasps are supposed to be ranked. 
    // select a target grasp pose from an array.
    geometry_msgs::msg::PoseStamped target_pose;
    if (!select_target_pose(grasp_array_result, target_pose))
    {
      RCLCPP_INFO(this->get_logger(),"no grasps exexcutable, exsiting...");
      return;
    }
    

    if (!open_gripper()) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"opened gripper successfully! Next step: move to target pose");

    // if (!move_to_pose(example_target_pose)) {cleanup();return;}
    if (!move_to_pose(target_pose)) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"Moved to target grasp pose successfully! Next step: close gripper");

    if (!close_gripper()) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"closed gripper successfully! Next step: move straight up");

    // if (!move_straight_up(example_target_pose)) {cleanup();return;}
    if (!move_to_pose(target_pose)) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"move straight up successfully! Next step: test grasp");

    if (!test_grasp()) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"tested grasp successfully! Next step: open gripper");

    if (!open_gripper()) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"opened gripper successfully!");

  }

  void test_function_cb(const std_srvs::srv::Empty::Request::SharedPtr request,
                      std_srvs::srv::Empty::Response::SharedPtr response)
  {
    cleanup();

    if (!move_to_start()){cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"Moved to start successfully! Waiting for the grasps");

    RCLCPP_INFO_STREAM(this->get_logger(),"time is: "<<std::to_string(rclcpp::Clock{}.now().seconds()));
    
    return;
  }


}; //GraspMoveit

int main(int argc, char ** argv)
{
  
  rclcpp::init(argc, argv);
  auto grasp_moveit = std::make_shared<GraspMoveit>();
  // rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(grasp_moveit);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
