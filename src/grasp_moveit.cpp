#include <memory>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <math.h>

using std::placeholders::_1;
using std::placeholders::_2;
const std::string MOVE_GROUP = "panda_arm";


class GraspMoveit : public rclcpp::Node
{
  public:
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr grasp_service, move_to_start_service;
  sensor_msgs::msg::JointState start_joint_state;

  geometry_msgs::msg::Pose target_pose;


  GraspMoveit(): Node("grasp_moveit"), move_group_interface(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
  { 
    // for the test
    target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
    }();

    // create start state
    std::string name_prefix="panda_joint";
    for(int i=1;i<=7;i++)
    {
      start_joint_state.name.push_back(name_prefix+std::to_string(i));
    }
    start_joint_state.position = std::vector<double,std::allocator<double>>{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};

    // initialize services
    grasp_service=this->create_service<std_srvs::srv::Empty>("execute_grasp", std::bind(&GraspMoveit::execute_grasp_cb,this,_1,_2));
    move_to_start_service=this->create_service<std_srvs::srv::Empty>("move_to_start", std::bind(&GraspMoveit::move_to_start_cb,this,_1,_2));
    RCLCPP_INFO(this->get_logger(), "initialization successful! waiting for the service /execute_grasp_service call");
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
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    if (move_group_interface.setJointValueTarget(start_joint_state))
    {
      auto success = move_group_interface.plan(msg);

      if(success) {
        move_group_interface.execute(msg);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Planing failed!");
      }
      return true;
    }
    return false;
  }

  void cleanup()
  {
    move_group_interface.clearPoseTarget();
  }

  bool move_to_pose(const geometry_msgs::msg::Pose &target_pose)
  {
    
    auto pre_grasp_pose = target_pose; // TODO
    move_group_interface.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto success = move_group_interface.plan(msg);

    // Execute the plan
    if(success) {
      if(ask_continue())
        {move_group_interface.execute(msg);}
      else {RCLCPP_INFO(this->get_logger(),"stop executing!");}
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planing failed!");
    }

  }

  bool close_gripper(){}
  bool open_gripper(){}
  bool test_grasp(){}

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
    RCLCPP_INFO(this->get_logger(),"Moved to start successfully!");

    if (!move_to_pose(target_pose)) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"Moved to grasp pose successfully!");

    if (!close_gripper()) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"closed gripper successfully!");

    if (!test_grasp()) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"tested grasp successfully!");

    if (!open_gripper()) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"opened gripper successfully!");

  }
}; //GraspMoveit

int main(int argc, char ** argv)
{
  
  rclcpp::init(argc, argv);
  auto grasp_moveit = std::make_shared<GraspMoveit>();
  rclcpp::spin(grasp_moveit);
  rclcpp::shutdown();

  return 0;
}
