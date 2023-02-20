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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// #include <moveit/pointcloud_octomap_updater>

#include <std_srvs/srv/empty.hpp>
#include <grasp_srv/srv/grasp_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <geometry_msgs/msg/transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"




// using std::placeholders::_1;
// using std::placeholders::_2;

using franka_msgs::action::Grasp;
using franka_msgs::action::Move;
const double pre_grasp_offset_z = -0.075; //meter
const std::string MOVE_GROUP = "panda_arm";
const std::string END_EFFECTOR_FRAME = "panda_hand_tcp";



class GraspMoveit : public rclcpp::Node
{
  public:
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr exec_grasp_service, move_to_start_service, test_function_service, test_grasp_service, move_up_service;
  rclcpp::Client<grasp_srv::srv::GraspArray>::SharedPtr grasp_client;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;
  sensor_msgs::msg::JointState start_joint_state, rot_right, rot_left;  
  sensor_msgs::msg::PointCloud2 point_cloud_collision;


  std::shared_ptr<occupancy_map_monitor::OccupancyMapMonitor> occ_moni;
  
  geometry_msgs::msg::Pose example_target_pose;

  rclcpp_action::Client<Grasp>::SharedPtr gripper_grasp_action;
  rclcpp_action::Client<Move>::SharedPtr gripper_move_action;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr empty_pc_pub;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> target_pose_broadcaster_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;



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

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    target_pose_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    

    geometry_msgs::msg::TransformStamped world2robot;
    world2robot.header.stamp = this->get_clock()->now();
    world2robot.header.frame_id = "world";
    world2robot.child_frame_id = "panda_table_link";

    world2robot.transform.rotation.x=0;
    world2robot.transform.rotation.y=0;
    world2robot.transform.rotation.z=0;
    world2robot.transform.rotation.w=1;
    world2robot.transform.translation.x=0;
    world2robot.transform.translation.y=0;
    world2robot.transform.translation.z=0;

    geometry_msgs::msg::TransformStamped robot2marker;
    robot2marker.header.stamp = this->get_clock()->now();
    robot2marker.header.frame_id = "aruco_marker_frame_windowed";
    robot2marker.child_frame_id = "world";

    robot2marker.transform.rotation.x=0;
    robot2marker.transform.rotation.y=0;
    robot2marker.transform.rotation.z=0;
    robot2marker.transform.rotation.w=1;
    robot2marker.transform.translation.x=-0.24;
    robot2marker.transform.translation.y=0.200;
    robot2marker.transform.translation.z=-0.00;

    
    tf_static_broadcaster_->sendTransform(world2robot);
    tf_static_broadcaster_->sendTransform(robot2marker);
    
    

    // create predefined states
    std::string name_prefix="panda_joint";
    std::vector<std::string> joints_name;
    for(int i=1;i<=7;i++)
    {
      joints_name.push_back(name_prefix+std::to_string(i));
    }
    start_joint_state.name=joints_name;
    start_joint_state.position = std::vector<double,std::allocator<double>>{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};

    rot_right.name=joints_name;
    rot_right.position = std::vector<double,std::allocator<double>>{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, 2.25};

    rot_left.name =joints_name;
    rot_left.position = std::vector<double,std::allocator<double>>{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, -0.84};

    // cb group
    callback_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);

    // initialize services and client
    exec_grasp_service=this->create_service<std_srvs::srv::Empty>("execute_grasp", std::bind(&GraspMoveit::execute_grasp_cb,this,std::placeholders::_1,std::placeholders::_2));
    move_to_start_service=this->create_service<std_srvs::srv::Empty>("move_to_start", std::bind(&GraspMoveit::move_to_start_cb,this,std::placeholders::_1,std::placeholders::_2));
    test_grasp_service=this->create_service<std_srvs::srv::Empty>("test_grasp", std::bind(&GraspMoveit::test_grasp_cb,this,std::placeholders::_1,std::placeholders::_2));
    point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud_collision",10,std::bind(&GraspMoveit::point_cloud_cb, this, std::placeholders::_1));
    // test_function_service=this->create_service<std_srvs::srv::Empty>("test", std::bind(&GraspMoveit::test_function_cb,this,std::placeholders::_1,std::placeholders::_2));
    grasp_client=this->create_client<grasp_srv::srv::GraspArray>("grasp_array", rmw_qos_profile_services_default, callback_group_);

    // pub
    empty_pc_pub = this->create_publisher<std_msgs::msg::Empty>("empty_pc", 10);

    // initialize actions
    gripper_move_action=rclcpp_action::create_client<Move>(this,"/panda_gripper/move");
    gripper_grasp_action=rclcpp_action::create_client<Grasp>(this,"/panda_gripper/grasp");
    

    move_group_interface.startStateMonitor();
    move_group_interface.setEndEffectorLink(END_EFFECTOR_FRAME);
    RCLCPP_INFO_STREAM(this->get_logger(), "end effector link is: "<<move_group_interface.getEndEffectorLink());
    RCLCPP_INFO_STREAM(this->get_logger(), "goal pos tolerance is : "<<move_group_interface.getGoalPositionTolerance());
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
    move_group_interface.setMaxVelocityScalingFactor(0.1);
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
    // for (auto name : conames)
    // {
    //   RCLCPP_INFO_STREAM(this->get_logger(), "collision object name: "<< name);
    // }
    planning_scene_interface.removeCollisionObjects(conames);
    
    // std::vector< moveit_msgs::msg::CollisionObject> empty_vec;
    // planning_scene_interface.applyCollisionObject(empty_vec);
    // the reason to use applyCollisionObject() rather than removeCollisionObjects() is the latter is asynchronous;
    // http://docs.ros.org/en/noetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1PlanningSceneInterface.html

    move_group_interface.clearPoseTarget();
    return;
  }

  bool move_to_pose(const geometry_msgs::msg::PoseStamped &target_pose, const moveit::planning_interface::MoveGroupInterface::Plan &plan_pre_grasp)
  {
    // Execute the plan
    if(!ask_continue()){return false;}
    move_group_interface.execute(plan_pre_grasp);
    // rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(this->get_logger(),"Moved to pre grasp pose successfully!");
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = plan_cartesian_path(target_pose, trajectory);
    
    // if (!success_target_pose){return false;}
    if(fraction==-1.0){return false;}
    if(!ask_continue()){return false;}
    move_group_interface.execute(trajectory);
    return true;
  }

  double plan_cartesian_path(const geometry_msgs::msg::PoseStamped& target_pose, moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose.pose);

    move_group_interface.setPoseReferenceFrame(target_pose.header.frame_id);
    double fraction = move_group_interface.computeCartesianPath(waypoints,
                                                0.005, // eef step
                                                0.00,    //jump threshol
                                                trajectory);
    
    RCLCPP_INFO_STREAM(this->get_logger(),"fraction is: "<<fraction);

    return fraction;
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
    goal.force=10;
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

  bool move_straight_up(const geometry_msgs::msg::PoseStamped &target_pose, const geometry_msgs::msg::TransformStamped &transform_cam_to_marker)
  {
    // move_group_interface.setStartStateToCurrentState();
    // auto higher_pose = move_group_interface.getCurrentPose().pose;
    geometry_msgs::msg::PoseStamped target_pose_marker_frame;

    tf2::doTransform(target_pose, target_pose_marker_frame, transform_cam_to_marker);

    // RCLCPP_INFO_STREAM(this->get_logger(), "target pose marker frame: "<< target_pose_marker_frame.header.frame_id);

    // auto higher_pose = target_pose_marker_frame;
    // higher_pose.pose.position.z += 0.10; // First move up (z)

    // // moveit_msgs::msg::RobotTrajectory trajectory = plan_cartesian_path(higher_pose);

    // move_group_interface.setPoseTarget(higher_pose);
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // auto success = move_group_interface.plan(plan);
    // if(!success){return false;}
    // if(!ask_continue()){return false;}
    // move_group_interface.execute(plan);
    // // move_group_interface.execute(trajectory);

    // return true;


    std::vector<geometry_msgs::msg::PoseStamped> higher_poses;
    // for(double i=0.10;i==0.05;i-=0.01)
    // {
    //   geometry_msgs::msg::PoseStamped up_pose = target_pose_marker_frame;
    //   up_pose.pose.position.z += i;
    //   higher_poses.push_back(up_pose);
    // }
    geometry_msgs::msg::PoseStamped higher_pose1 = target_pose_marker_frame;
    geometry_msgs::msg::PoseStamped higher_pose2 = target_pose_marker_frame;
    higher_pose1.pose.position.z += 0.10;
    higher_pose2.pose.position.z += 0.05;
    higher_poses.push_back(higher_pose1);
    higher_poses.push_back(higher_pose2);

    RCLCPP_INFO_STREAM(this->get_logger(), "higer poses: "<<higher_poses.size());

    for(auto higher_pose: higher_poses)
    {
      move_group_interface.setPoseTarget(higher_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto success = move_group_interface.plan(plan);
      if(!success){continue;}
      if(!ask_continue()){return false;}
      move_group_interface.execute(plan);
      return true;
    }

    RCLCPP_INFO(this->get_logger(), "cannot move staright up! ");
    return false;

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
    
    move_group_interface.setMaxVelocityScalingFactor(0.5);
    // turn left
    // move_group_interface.setStartStateToCurrentState();
    move_group_interface.setJointValueTarget(rot_left);
    auto success_left = move_group_interface.plan(plan);
    if(!success_left){move_group_interface.setMaxVelocityScalingFactor(0.1);return false;}
    if(!ask_continue()){move_group_interface.setMaxVelocityScalingFactor(0.1);return false;}
    move_group_interface.execute(plan);

    // go back to start state
    // move_group_interface.setStartStateToCurrentState();
    move_group_interface.setJointValueTarget(start_joint_state);
    auto success_start = move_group_interface.plan(plan);
    if(!success_start){move_group_interface.setMaxVelocityScalingFactor(0.1);return false;}
    if(!ask_continue()){move_group_interface.setMaxVelocityScalingFactor(0.1);return false;}
    move_group_interface.execute(plan);

    move_group_interface.setMaxVelocityScalingFactor(0.1); // default
    
    return true;
  }


  bool select_target_pose(const std::vector<geometry_msgs::msg::PoseStamped> &grasp_array, geometry_msgs::msg::PoseStamped &target_pose, moveit::planning_interface::MoveGroupInterface::Plan &plan)
  {
    
    for(auto grasp_pose : grasp_array)
    {
      geometry_msgs::msg::TransformStamped grasp_pose_transform;
      grasp_pose_transform.header.stamp = this->get_clock()->now();
      grasp_pose_transform.header.frame_id=grasp_pose.header.frame_id;
      grasp_pose_transform.child_frame_id="grasp_pose_frame";

      grasp_pose_transform.transform.rotation = grasp_pose.pose.orientation;
      grasp_pose_transform.transform.translation.x = grasp_pose.pose.position.x;
      grasp_pose_transform.transform.translation.y = grasp_pose.pose.position.y;
      grasp_pose_transform.transform.translation.z = grasp_pose.pose.position.z;

      target_pose_broadcaster_->sendTransform(grasp_pose_transform);

      geometry_msgs::msg::TransformStamped rotation;
      rotation.transform.rotation = grasp_pose.pose.orientation;

      geometry_msgs::msg::Vector3Stamped pre_grasp_offset, pre_grasp_offset_cam;
      pre_grasp_offset.vector.x=0.;
      pre_grasp_offset.vector.y=0.;
      pre_grasp_offset.vector.z=pre_grasp_offset_z;

      tf2::doTransform(pre_grasp_offset, pre_grasp_offset_cam, rotation);

      // RCLCPP_INFO_STREAM(this->get_logger(),"rotation"<<rotation.transform.rotation.w<<rotation.transform.rotation.x<<rotation.transform.rotation.y<<rotation.transform.rotation.z);
      // RCLCPP_INFO_STREAM(this->get_logger(),"x y z: "<<pre_grasp_offset_cam.vector.x<<pre_grasp_offset_cam.vector.y<<pre_grasp_offset_cam.vector.z);
      
      geometry_msgs::msg::PoseStamped pre_grasp_pose = grasp_pose;
      pre_grasp_pose.pose.position.x+=pre_grasp_offset_cam.vector.x;
      pre_grasp_pose.pose.position.y+=pre_grasp_offset_cam.vector.y;
      pre_grasp_pose.pose.position.z+=pre_grasp_offset_cam.vector.z;

      move_group_interface.setPoseTarget(pre_grasp_pose);
      if (move_group_interface.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        target_pose=grasp_pose;
        RCLCPP_INFO(this->get_logger(),"select one grasp successfully!");
        return true;
      }
      // double fraction = plan_cartesian_path(pre_grasp_pose, trajectory_pre_grasp);
      // if (fraction!=-1.0)
      // {
      //   target_pose=grasp_pose;
      //   return true;
      // }
    }

    return false;
  }

  private:

  void point_cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr pcd)
  {

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "kinect2_ir_optical_frame";
    collision_object.id = "point_cloud_collision";
    collision_object.operation = collision_object.ADD;

    const size_t number_of_points = pcd->height * pcd->width;
    sensor_msgs::PointCloud2Iterator<float> iter_x(*pcd, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pcd, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pcd, "z");
    for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z)
    {
      double x = *iter_x;
      double y = *iter_y;
      double z = *iter_z;

      shape_msgs::msg::SolidPrimitive primitive;

      // Define the size of the box in meters
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 0.002;
      primitive.dimensions[primitive.BOX_Y] = 0.002;
      primitive.dimensions[primitive.BOX_Z] = 0.002;

      geometry_msgs::msg::Pose point_pose;
      point_pose.orientation.w = 1.0;
      point_pose.position.x = x;
      point_pose.position.y = y;
      point_pose.position.z = z;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(point_pose);
    }
    planning_scene_interface.applyCollisionObject(collision_object);

    RCLCPP_INFO(this->get_logger(),"add collision object!");
  }

  void test_grasp_cb(const std_srvs::srv::Empty::Request::SharedPtr request,
                      std_srvs::srv::Empty::Response::SharedPtr response)
  {
    test_grasp();
    open_gripper();
  }
  void move_to_start_cb(const std_srvs::srv::Empty::Request::SharedPtr request,
                      std_srvs::srv::Empty::Response::SharedPtr response)
  {
    move_to_start();
  }


  void execute_grasp_cb(const std_srvs::srv::Empty::Request::SharedPtr request,
                      std_srvs::srv::Empty::Response::SharedPtr response)
  {
    // all actions are registered in the response callback, the reason is to avoid deadlock
    auto send_request = std::make_shared<grasp_srv::srv::GraspArray::Request>();
    auto future = grasp_client->async_send_request(send_request, std::bind(&GraspMoveit::response_callback, this, std::placeholders::_1));

  }

  void response_callback(rclcpp::Client<grasp_srv::srv::GraspArray>::SharedFuture future)
  { 
    if (!move_to_start()){cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"Moved to start successfully! Waiting for the grasps");

    std::vector<geometry_msgs::msg::PoseStamped> grasp_array_result = future.get()->grasp_array;

    // grasps are supposed to be ranked. 
    // select a target grasp pose from an array.
    geometry_msgs::msg::PoseStamped target_pose;
    geometry_msgs::msg::TransformStamped transform_cam_to_marker;
    moveit::planning_interface::MoveGroupInterface::Plan plan_pre_grasp;
    
    if (!select_target_pose(grasp_array_result, target_pose, plan_pre_grasp))
    {
      RCLCPP_INFO(this->get_logger(),"no grasps exexcutable, exsiting...");
      return;
    }

    if(!tf_buffer->canTransform("aruco_marker_frame_windowed",
                                target_pose.header.frame_id,
                                tf2::TimePointZero
                                ))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "No transformation: ");
      return ;
    }

    try{
      transform_cam_to_marker = tf_buffer->lookupTransform(
                                "aruco_marker_frame_windowed",
                                target_pose.header.frame_id,
                                tf2::TimePointZero
                                  );
      
    }
    catch(tf2::TransformException &e){
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error in lookuptransform: "<<e.what());
      return ;
    }
    
    if (!open_gripper()) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"opened gripper successfully! Next step: move to target pose");


    // if (!move_to_pose(example_target_pose)) {cleanup();return;}
    if (!move_to_pose(target_pose, plan_pre_grasp)) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"Moved to target grasp pose successfully! Next step: close gripper");

    planning_scene_interface.removeCollisionObjects(std::vector<std::string>{"point_cloud_collision"});
    RCLCPP_INFO(this->get_logger(),"collision object removed!");

    rclcpp::sleep_for(std::chrono::seconds(2));

    if (!close_gripper()) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"closed gripper successfully! Next step: move straight up");

    if (!move_straight_up(target_pose, transform_cam_to_marker)) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"moved straight up! Next step: test grasp");

    if (!test_grasp()) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"tested grasp successfully! Next step: open gripper");

    if (!open_gripper()) {cleanup();return;}
    RCLCPP_INFO(this->get_logger(),"opened gripper successfully!");
    RCLCPP_INFO(this->get_logger(),"grasp finished, waiting for the next call...");

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
  rclcpp::executors::MultiThreadedExecutor executor;
  // rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(grasp_moveit);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
