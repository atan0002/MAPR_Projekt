#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ompl-1.6/ompl/geometric/SimpleSetup.h>
#include <ompl-1.6/ompl/geometric/planners/rrt/RRTstar.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
// #include <moveit/moveit_msgs/DisplayTrajectory>
// #include <moveit/moveit_msgs/RobotTrajectory>

#include <ompl-1.6/ompl/base/spaces/SE3StateSpace.h>
#include <ompl-1.6/ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl-1.6/ompl/base/spaces/RealVectorBounds.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <typeinfo>
#include "geometry_msgs/msg/pose_stamped.h"
#include "nav_msgs/msg/path.hpp"
#include <vector>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>



namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace rvt = rviz_visual_tools;


void plan_visualize(auto node)
{
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, 
                                                                      move_group_interface.getRobotModel()};

    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    auto const draw_trajectory_tool_path = [&moveit_visual_tools,
                                     jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator")](auto const trajectory)
                                     {moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

    std::map<std::string, double> target_joint_vals;
    target_joint_vals["shoulder_pan_joint"] =  0.0;
    target_joint_vals["shoulder_lift_joint"] = 0.785;
    target_joint_vals["elbow_joint"] = 0.0;   
    target_joint_vals["wrist_1_joint"] = 0.0; 
    target_joint_vals["wrist_2_joint"] = 0.0;
    target_joint_vals["wrist_3_joint"] = -0.785;

    move_group_interface.setJointValueTarget(target_joint_vals);
    // move_group_interface.setPoseTarget(target_pose);
    auto const [success, plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success)
    {
        draw_trajectory_tool_path(plan.trajectory_);
        std::cout<<"Executing!"<<std::endl;
        move_group_interface.execute(plan);
    }
    else
        std::cout<<"Planning failed!"<<std::endl;
    
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "plan_trajectory",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("plan_trajectory");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]()
                             { executor.spin(); });

    static const std::vector<std::string> CONTROLLERS(1, "joint_trajectory_controller");

    const std::string PLANNING_GROUP = "ur";

    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    robot_state->setToDefaultValues();
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("ur_manipulator");

    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    auto move_group_interface =  moveit::planning_interface::MoveGroupInterface(node, "ur_manipulator");
    move_group_interface.setMaxAccelerationScalingFactor(0.01);

    std::vector<double> group_variable_values;
    move_group_interface.getCurrentState()->copyJointGroupPositions(move_group_interface.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_interface.getName()), group_variable_values);

    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    robot_state->setJointGroupPositions(joint_model_group, group_variable_values);
    RCLCPP_INFO(logger, "########## Joint states - start ##########");
    //Joint states - start
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // planowanie sciezki
    plan_visualize(node);

    //Joint states - goal
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    robot_state->setJointGroupPositions(joint_model_group, group_variable_values);
    RCLCPP_INFO(logger, "########## Joint states - goal ##########");
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    rclcpp::shutdown();
    spinner.join();

    return 0;
}