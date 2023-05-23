#include <memory>
#include <iostream>
#include <typeinfo>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/srv/empty.hpp>
// #include "moveit2_interface/srv/move_cartesian.hpp"


using moveit::planning_interface::MoveGroupInterface;
MoveGroupInterface *move_group_interface;
const std::string MOVE_GROUP="panda_arm";
auto LOGGER = rclcpp::get_logger("precheck");


class PreCheck : public rclcpp::Node
{
    public:
    PreCheck(): Node("precheck_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        RCLCPP_INFO(LOGGER, "start going to points on hemisphere");
    }

};

void move_to_pose(const std_srvs::srv::Empty::Request::SharedPtr request, 
                const std_srvs::srv::Empty::Response::SharedPtr response)
{
    RCLCPP_INFO(LOGGER, "move to pose");
}
int main(int argc, char * argv[])
{

    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    // auto const move_group_node = std::make_shared<rclcpp::Node>("moveit2_interface_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const move_group_node = std::make_shared<PreCheck>();
    // We spin up a SingleThreadedExecutor for the current state monitor to get information about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);                       //Executer: https://docs.ros.org/en/foxy/Concepts/About-Executors.html
    std::thread thread([&executor]() { executor.spin();});

    // Bind move_cartesian service to the node
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service = move_group_node->create_service<std_srvs::srv::Empty>("move_to_pose", &move_to_pose);

    // Create a ROS logger

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;

    // class can be easily set up using just the name of the planning group you would like to control and plan for.
    static const std::string PLANNING_GROUP = "panda_arm";
    move_group_interface = new MoveGroupInterface(move_group_node, PLANNING_GROUP);

    // move_group_interface->startStateMonitor();
    // // check
    auto state = move_group_interface->getCurrentState();
    auto pose =  move_group_interface->getCurrentPose(); 

    RCLCPP_INFO_STREAM(rclcpp::get_logger("moveit2_interface"), "pose: "<<pose.pose.position.x);

    thread.join();
}