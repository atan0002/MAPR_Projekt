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
// #include <moveit/moveit_msgs/DisplayTrajectory>
// #include <moveit/moveit_msgs/RobotTrajectory>
#include <moveit/move_group_interface/move_group_interface.h>
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
#include <moveit/planning_scene/planning_scene.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace rvt = rviz_visual_tools;
using moveit::planning_interface::MoveGroupInterface;


int dim;

    /// max step length
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

    
    // return true;
       // Cast the state to RealVectorStateSpace::StateType
    const auto* stateVec = state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Extract the joint values from the state
    double joint1 = (*stateVec)[0];
    double joint2 = (*stateVec)[1];
    double joint3 = (*stateVec)[2];
    double joint4 = (*stateVec)[3];
    double joint5 = (*stateVec)[4];
    double joint6 = (*stateVec)[5];

    // Perform collision checking based on your environment or specific constraints
    // Return true if the state is collision-free, false otherwise

    // Example: Check if any joint is outside its valid range
    if (joint1 < -M_PI  || joint1 >  M_PI ||
        joint2 <  -M_PI  || joint2 >  M_PI  ||
        joint3 <  -M_PI  || joint3 >  M_PI ||
        joint4 < -M_PI || joint4 >  M_PI ||
        joint5 < -M_PI || joint5 >  M_PI ||
        joint6 < -M_PI || joint6 > M_PI)
    {
        return false; // Invalid state due to joint limits violation
    }

    // Example: Check for collision with obstacles in the environment
    // You would need to use your own collision detection algorithm or library

    // Return true if the state is collision-free
    return true; //colisionchecker

}



void configure(){

    dim = 3;//2D problem
    maxStepLength = 0.1;// max step length

    coordXBound.reset(new ob::RealVectorBounds(dim-1));
    coordXBound->setLow(-1.0);
    coordXBound->setHigh(13.0);
    
    coordYBound.reset(new ob::RealVectorBounds(dim-1));
    coordYBound->setLow(-5.0);
    coordYBound->setHigh(5.0);


    coordZBound.reset(new ob::RealVectorBounds(dim-1));
    coordZBound->setLow(-5.0);
    coordZBound->setHigh(10.0);

    auto coordX(std::make_shared<ob::RealVectorStateSpace>(dim-1));
    auto coordY(std::make_shared<ob::RealVectorStateSpace>(dim-1));
    auto coordZ(std::make_shared<ob::RealVectorStateSpace>(dim-1));
    space = coordX +coordY + coordZ;


    coordX->setBounds(*coordXBound.get());

    // create bounds for the y axis
    coordY->setBounds(*coordYBound.get());

    coordZ->setBounds(*coordZBound.get());

       // define the start position
    start.reset(new ob::ScopedState<>(space));
    (*start.get())[0]=0.0;
    (*start.get())[1]=-2.5;
    (*start.get())[2]=2.5;
//    start.get()->random();

    // define the goal position
    goal.reset(new ob::ScopedState<>(space));
    (*goal.get())[0]=12.0;
    (*goal.get())[1]=-4.0;
    (*goal.get())[2]=-2.0;
//    goal.get()->random();




}





ob::PathPtr plan( std::vector<double> start_pos){

    auto space(std::make_shared<ob::RealVectorStateSpace>(6));
    ob::RealVectorBounds bounds(6);
    bounds.setLow(-M_PI);
    bounds.setHigh( M_PI);
    space->setBounds(0.0, 2.0 * M_PI); 
 
    space->setBounds(bounds);
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setStateValidityChecker(isStateValid);
    

    ob::ScopedState<> start(space);
    start[0] = start_pos[0]; // Joint 1 initial configuration
    start[1] = start_pos[1]; // Joint 2 initial configuration
    start[2] = start_pos[2]; // Joint 3 initial configuration
    start[3] = start_pos[3]; // Joint 4 initial configuration
    start[4] = start_pos[4]; // Joint 5 initial configuration
    start[5] = start_pos[5]; // Joint 6 initial configuration

    ob::ScopedState<> goal(space);
    goal[0] =0.0; // Joint 1 goal configuration
    goal[1] = -1.57;        // Joint 2 goal configuration
    goal[2] = 0.0;        // Joint 3 goal configuration
    goal[3] = -1.57;        // Joint 4 goal configuration
    goal[4] = 0.0;        // Joint 5 goal configuration
    goal[5] = 0.0;        // Joint 6 goal configuration;

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    
    pdef->setStartAndGoalStates(start, goal);

    auto planner(std::make_shared<og::RRTConnect>(si));
    planner->setProblemDefinition(pdef);

    planner->setup();

    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
 
        // print the path to screen
        path->print(std::cout);

                
        return path;
    }



}


std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> extractPath(ob::PathPtr path,std::vector<double> start_pos ){

    moveit_msgs::msg::DisplayTrajectory display_trajectory;


    
    const auto *path_ = path.get()->as<og::PathGeometric>();
    
    auto displayTrajectoryMsg = std::make_shared<moveit_msgs::msg::DisplayTrajectory>();

    auto robotStateMsg = std::make_shared<moveit_msgs::msg::RobotState>();
    auto jointStateMsg = std::make_shared<sensor_msgs::msg::JointState>();
    auto  PlanMsg= std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>();
    
    std::cout<<start_pos[1]<<std::endl;

    
    jointStateMsg->position ={start_pos[0],start_pos[1],start_pos[2],start_pos[3],start_pos[4],start_pos[5]};


    jointStateMsg->name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    PlanMsg->planning_time_=5;


    robotStateMsg->joint_state =*jointStateMsg;

  

    // path_->getStateCount()
    for(unsigned int i=0; i<2; ++i){

        const ob::State* state = path_->getState(i);
        // moveit_msgs::msg::RobotTrajectory robot_trajectory;
        // trajectory_msgs::msg::JointTrajectoryPoint traj_point;
        
        auto robotTrajectoryMsg = std::make_shared<moveit_msgs::msg::RobotTrajectory>();
        auto jointTrajectoryMsg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
        
        // auto robotStateMsg = std::make_shared<moveit_msgs::msg::RobotState>();
        
        // jointTrajectoryMsg->header.stamp = node->get_clock()->now(); 

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

        std::cout<< joint1<<std::endl;

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

        jointTrajectoryMsg->joint_names={"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint","wrist_2_joint","wrist_3_joint"};


        
       
        robotTrajectoryMsg->joint_trajectory = *jointTrajectoryMsg;

        // displayTrajectoryMsg->trajectory_start = *robotStateMsg;
        // displayTrajectoryMsg->trajectory.push_back(*robotTrajectoryMsg);
        PlanMsg->start_state_=*robotStateMsg;
        PlanMsg->trajectory_=*robotTrajectoryMsg;
     



        // auto robotStateMsg = std::make_shared<moveit_msgs::msg::RobotState>();
        // robotStateMsg->joint_state.position= {joint1,joint2,joint3,joint4,joint5,joint6};
        jointStateMsg->position ={joint1,joint2,joint3,joint4,joint5,joint6};// {0.0,0.0,0.0,0.0,0.0,0.0};//{joint1,joint2,joint3,joint4,joint5,joint6};

        jointStateMsg->name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint","wrist_2_joint","wrist_3_joint"};
        robotStateMsg->joint_state =*jointStateMsg;
        std::cout<< "##########test3######"<<std::endl;
       
        // jakis problem z przypisaniem wartości joint stateow do debugu
        // traj_point.positions.push_back(joint1);
        // robot_trajectory.joint_trajectory=traj_point;
        // std::cout<< "##########test4######"<<std::endl;
        // display_trajectory.trajectory.push_back(robot_trajectory);
        
    }

    return PlanMsg;

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


  
    const std::string PLANNING_GROUP = "ur";
    // const std::string ROBOT_DESC="robot_description";
    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    // robot_state->setToDefaultValues();

    planning_scene::PlanningScene planning_scene(robot_model);
    moveit::core::RobotState &current_state = planning_scene.getCurrentStateNonConst();



   
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator"); //std::make_shared<MoveGroupInterface>(node, "ur_manipulator");

    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("ur_manipulator");
    //  planning_scene.getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

   


   





    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    // auto startStates=current_state.getJointPositions(joint_model_group)

  
    RCLCPP_INFO(logger, "##########test######");

    std::vector<double> group_variable_values;
    move_group_interface.getCurrentState()->copyJointGroupPositions(move_group_interface.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_interface.getName()), group_variable_values);
   
     for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), group_variable_values[i]);
    }

    moveit::core::RobotState start_state(move_group_interface.getRobotModel());

    start_state.setJointGroupPositions(move_group_interface.getName(), group_variable_values);
    move_group_interface.setStartState(start_state);

    std::vector<double> joint_values;
    robot_state->setJointGroupPositions(joint_model_group, group_variable_values);
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
   
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        RCLCPP_INFO(logger, "JointModelGroup Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }


    // move_group_interface.setStartState();



    // planowanie sciezki
    // auto moveit_visual_tools =moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,move_group_interface->getRobotModel()};
    // moveit_visual_tools.deleteAllMarkers();
    // moveit_visual_tools.loadRemoteControl();

    auto path=plan(group_variable_values);

    auto disTraj=extractPath(path,group_variable_values);
   
    RCLCPP_INFO(logger, "##########test2######");


    auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                             move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator")](
          auto const trajectory)
  { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

    draw_trajectory_tool_path(disTraj->trajectory_);
    move_group_interface.execute(*disTraj);
       


  
    //przeksztalcenie jej na wiadomosc do wizualizacji

    // move_group_interface->setMaxVelocityScalingFactor(0.05);
    // move_group_interface->setMaxAccelerationScalingFactor(0.05);

 
   
   // wizualizacja
//     moveit_visual_tools::MoveItVisualTools visual_tools{node,"base_link",rviz_visual_tools::RVIZ_MARKER_TOPIC,robot_model};
    
//     // visual_tools.loadRobotStatePub("/display_planned_path");
//     // visual_tools.enableBatchPublishing();
//     visual_tools.deleteAllMarkers();  // clear all old markers
//     // visual_tools.trigger();



//     visual_tools.loadRemoteControl();

//     // visual_tools.trigger();
//     rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 100);
//     display_publisher->publish(disTraj);
//     // async execute/execute robot_trajectory z move_group_interface
//    //for(int i =0; i<disTraj.trajectory.size();i++){
//     visual_tools.publishTrajectoryLine(disTraj.trajectory.back(), joint_model_group);

        
//     //}
//     visual_tools.trigger();

    // RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    // RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    // RCLCPP_INFO(logger, "Available Planning Groups:");
    // // std::copy(move_group_interface.getJointModelGroupNames().begin(), move_group_interface.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));


   
    // moveit_visual_tools.trigger();


    // auto const ok = static_cast<bool>(move_group_interface.execute(disTraj));

    // std::cout<<ok<<std::endl;
    // RCLCPP_INFO(logger, "Error code: %s", code);


    rclcpp::shutdown();
    spinner.join();
   




    return 0;
}