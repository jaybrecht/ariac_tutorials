#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class RobotCommander : public rclcpp::Node
{
public:
  RobotCommander();
  ~RobotCommander();

private:
  // MoveIt Interfaces 
  moveit::planning_interface::MoveGroupInterface floor_robot_;
  moveit::planning_interface::MoveGroupInterface ceiling_robot_;

  // ROS Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr floor_robot_move_home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ceiling_robot_move_home_srv_;

  // Service Callbacks
  void FloorRobotMoveHome(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void CeilingRobotMoveHome(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
};

RobotCommander::RobotCommander()
 : Node("robot_commander"),
  floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
  ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot")
{
  // Use upper joint velocity and acceleration limits
  floor_robot_.setMaxAccelerationScalingFactor(1.0);
  floor_robot_.setMaxVelocityScalingFactor(1.0);

  ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
  ceiling_robot_.setMaxVelocityScalingFactor(1.0);

  // Register services
  floor_robot_move_home_srv_ = create_service<std_srvs::srv::Trigger>(
    "/competitor/move_floor_robot_home", 
    std::bind(
      &RobotCommander::FloorRobotMoveHome, this,
      std::placeholders::_1, std::placeholders::_2));
  
  ceiling_robot_move_home_srv_ = create_service<std_srvs::srv::Trigger>(
    "/competitor/move_ceiling_robot_home", 
    std::bind(
      &RobotCommander::CeilingRobotMoveHome, this,
      std::placeholders::_1, std::placeholders::_2));
}

RobotCommander::~RobotCommander() 
{
  floor_robot_.~MoveGroupInterface();
  ceiling_robot_.~MoveGroupInterface();
}

void RobotCommander::FloorRobotMoveHome(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)req; // remove unused parameter warning
  floor_robot_.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_.plan(plan));

  if (success) {
    if (static_cast<bool>(floor_robot_.execute(plan))) {
      res->success = true;
    } else {
      res->success = false;
      res->message = "Trajectory execution failed";
    }
  } else {
    res->message = "Unable to generate trajectory";
    res->success = false;
  }
}

void RobotCommander::CeilingRobotMoveHome(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)req; // remove unused parameter warning
  ceiling_robot_.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(ceiling_robot_.plan(plan));

  if (success) {
    if (static_cast<bool>(ceiling_robot_.execute(plan))) {
      res->success = true;
    } else {
      res->success = false;
      res->message = "Trajectory execution failed";
    }
  } else {
    res->message = "Unable to generate trajectory";
    res->success = false;
  }
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto robot_commander = std::make_shared<RobotCommander>();
  rclcpp::spin(robot_commander);
  rclcpp::shutdown();
}