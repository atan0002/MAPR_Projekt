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



double maxStepLength;

    /// bounds for the x axis
std::shared_ptr<ompl::base::RealVectorBounds> coordXBound;

    /// bounds for the y axis
std::shared_ptr<ompl::base::RealVectorBounds> coordYBound;

std::shared_ptr<ompl::base::RealVectorBounds> coordZBound;

    /// start position
std::shared_ptr<ompl::base::ScopedState<>> start;

    /// goal position
std::shared_ptr<ompl::base::ScopedState<>> goal;

    /// search space
std::shared_ptr<ompl::base::StateSpace> space;


bool isStateValid(const ob::State *state){


    const auto* stateVec = state->as<ompl::base::RealVectorStateSpace::StateType>();


    double joint1 = (*stateVec)[0];
    double joint2 = (*stateVec)[1];
    double joint3 = (*stateVec)[2];
    double joint4 = (*stateVec)[3];
    double joint5 = (*stateVec)[4];
    double joint6 = (*stateVec)[5];

 
    if (joint1 <=-M_PI/2 || joint1 >=  M_PI/2 ||
        joint2 <=-M_PI/2 || joint2 >=  M_PI/2 ||
        joint3 <=-M_PI/2 || joint3 >=  M_PI/2 ||
        joint4 <=-M_PI/2 || joint4 >=  M_PI/2 ||
        joint5 <=-M_PI/2 || joint5 >=  M_PI/2 ||
        joint6 <=-M_PI/2 || joint6 >= M_PI/2)
    {
        return false; // Invalid state due to joint limits violation
    }
    return true;

}





ob::PathPtr plan(std::vector<double> start_,std::vector<double> goal_){

    auto space(std::make_shared<ob::RealVectorStateSpace>(6));
    ob::RealVectorBounds bounds(6);
    bounds.setLow(-M_PI/2);
    bounds.setHigh( M_PI/2);
    space->setBounds(-M_PI/2,  M_PI/2); 
 
    space->setBounds(bounds);
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setStateValidityChecker(isStateValid);

    ob::ScopedState<> start(space);
    start[0] = start_[0]; // Joint 1 initial configuration
    start[1] = start_[1]; // Joint 2 initial configuration
    start[2] = start_[2]; // Joint 3 initial configuration
    start[3] = start_[3]; // Joint 4 initial configuration
    start[4] = start_[4]; // Joint 5 initial configuration
    start[5] = start_[5]; // Joint 6 initial configuration

    ob::ScopedState<> goal(space);
    goal[0] =goal_[0]; // Joint 1 goal configuration
    goal[1] = goal_[1];        // Joint 2 goal configuration
    goal[2] =goal_[2];        // Joint 3 goal configuration
    goal[3] = goal_[3];        // Joint 4 goal configuration
    goal[4] = goal_[4];        // Joint 5 goal configuration
    goal[5] = goal_[5];        // Joint 6 goal configuration;

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    
    pdef->setStartAndGoalStates(start, goal);

    auto planner(std::make_shared<og::RRTConnect>(si));
    planner->setProblemDefinition(pdef);

    planner->setup();

    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
      
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        path->print(std::cout);

                
        return path;
    }



}

moveit_msgs::msg::DisplayTrajectory extractPath(ob::PathPtr path,std::vector<double> start_){

    moveit_msgs::msg::DisplayTrajectory display_trajectory;


    
    const auto *path_ = path.get()->as<og::PathGeometric>();
    
    auto displayTrajectoryMsg = std::make_shared<moveit_msgs::msg::DisplayTrajectory>();

    auto robotStateMsg = std::make_shared<moveit_msgs::msg::RobotState>();
    auto jointStateMsg = std::make_shared<sensor_msgs::msg::JointState>();
    jointStateMsg->position = {start_[0], start_[1], start_[2], start_[3],start_[4],start_[5]};

    jointStateMsg->name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint","wrist_2_joint","wrist_3_joint"};


    robotStateMsg->joint_state =*jointStateMsg;

    for(unsigned int i=0; i<path_->getStateCount(); ++i){

        const ob::State* state = path_->getState(i);
;
        
        auto robotTrajectoryMsg = std::make_shared<moveit_msgs::msg::RobotTrajectory>();
        auto jointTrajectoryMsg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    

        trajectory_msgs::msg::JointTrajectoryPoint point1; 
        trajectory_msgs::msg::JointTrajectoryPoint point2; 
        trajectory_msgs::msg::JointTrajectoryPoint point3; 
        trajectory_msgs::msg::JointTrajectoryPoint point4; 
        trajectory_msgs::msg::JointTrajectoryPoint point5; 
        trajectory_msgs::msg::JointTrajectoryPoint point6; 

        auto joint1 = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        auto joint2 = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        auto joint3 = state->as<ob::RealVectorStateSpace::StateType>()->values[2];
        auto joint4 = state->as<ob::RealVectorStateSpace::StateType>()->values[3];
        auto joint5 = state->as<ob::RealVectorStateSpace::StateType>()->values[4];
        auto joint6 = state->as<ob::RealVectorStateSpace::StateType>()->values[5];



        point1.positions.push_back(joint1);
        point2.positions.push_back(joint2);
        point3.positions.push_back(joint3);
        point4.positions.push_back(joint4);
        point5.positions.push_back(joint5);
        point6.positions.push_back(joint6);
        

        jointTrajectoryMsg->points.push_back(point1);
        jointTrajectoryMsg->points.push_back(point2);
        jointTrajectoryMsg->points.push_back(point3);
        jointTrajectoryMsg->points.push_back(point4);
        jointTrajectoryMsg->points.push_back(point5);
        jointTrajectoryMsg->points.push_back(point6);


        
       
        robotTrajectoryMsg->joint_trajectory = *jointTrajectoryMsg;

        displayTrajectoryMsg->trajectory_start = *robotStateMsg;
        displayTrajectoryMsg->trajectory.push_back(*robotTrajectoryMsg);


       
        jointStateMsg->position = {joint1,joint2,joint3,joint4,joint5,joint6};

        jointStateMsg->name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint","wrist_2_joint","wrist_3_joint"};
        robotStateMsg->joint_state =*jointStateMsg;
  
       
   
        
    }

    return *displayTrajectoryMsg;

}






int main(int argc, char *argv[]){

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
    
    RCLCPP_INFO(logger, argv[1]);

  
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
   
     for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), group_variable_values[i]);
    }


    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    robot_state->setJointGroupPositions(joint_model_group, group_variable_values);
    
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }





    std::vector<double> goal={0.0,1.57/2,0.0,0.0,-1.57/2};

    // planowanie sciezki
    auto path=plan(group_variable_values,goal);

  

    auto disTraj=extractPath(path,group_variable_values);


 
   
//    // wizualizacja
    moveit_visual_tools::MoveItVisualTools visual_tools{node,"shoulder_pan_joint",rviz_visual_tools::RVIZ_MARKER_TOPIC,robot_model};

    visual_tools.deleteAllMarkers();  // clear all old markers
  

    visual_tools.loadRemoteControl();
    
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 2000);
    display_publisher->publish(disTraj);

      for(int i =0; i<disTraj.trajectory.size();i++){
        
        visual_tools.publishTrajectoryLine(disTraj.trajectory[i], joint_model_group);
      
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();
        visual_tools.trigger();

      
    }

   
    


    rclcpp::shutdown();
    spinner.join();




    return 0;
}