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

#include <ompl-1.6/ompl/base/spaces/SE3StateSpace.h>
#include <ompl-1.6/ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl-1.6/ompl/base/spaces/RealVectorBounds.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <typeinfo>
#include "geometry_msgs/msg/pose_stamped.h"
#include "nav_msgs/msg/path.hpp"
#include <vector>


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace rvt = rviz_visual_tools;


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
    if (joint1 < 0.0 || joint1 > 2.0 * M_PI ||
        joint2 < 0.0 || joint2 > 2.0 * M_PI ||
        joint3 < 0.0 || joint3 > 2.0 * M_PI ||
        joint4 < 0.0 || joint4 > 2.0 * M_PI ||
        joint5 < 0.0 || joint5 > 2.0 * M_PI ||
        joint6 < 0.0 || joint6 > 2.0 * M_PI)
    {
        return false; // Invalid state due to joint limits violation
    }

    // Example: Check for collision with obstacles in the environment
    // You would need to use your own collision detection algorithm or library

    // Return true if the state is collision-free
    return true;

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





ob::PathPtr plan(){

    auto space(std::make_shared<ob::RealVectorStateSpace>(6));
    ob::RealVectorBounds bounds(6);
    bounds.setLow(-M_PI);
    bounds.setHigh( M_PI);
    space->setBounds(0.0, 2.0 * M_PI); 
 
    space->setBounds(bounds);
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setStateValidityChecker(isStateValid);

    ob::ScopedState<> start(space);
    start[0] = 0.0; // Joint 1 initial configuration
    start[1] = 0.0; // Joint 2 initial configuration
    start[2] = 0.0; // Joint 3 initial configuration
    start[3] = 0.0; // Joint 4 initial configuration
    start[4] = 0.0; // Joint 5 initial configuration
    start[5] = 0.0; // Joint 6 initial configuration

    ob::ScopedState<> goal(space);
    goal[0] = M_PI / 2.0; // Joint 1 goal configuration
    goal[1] = 0.0;        // Joint 2 goal configuration
    goal[2] = 0.0;        // Joint 3 goal configuration
    goal[3] = 0.0;        // Joint 4 goal configuration
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


moveit_msgs::msg::DisplayTrajectory extractPath(ob::PathPtr path){

    moveit_msgs::msg::DisplayTrajectory display_trajectory;


    
    const auto *path_ = path.get()->as<og::PathGeometric>();
    // og::PathGeometric path_( dynamic_cast< const og::PathGeometric& >( path));

    // std::cout<<path_->getState(0)<<std::endl;

    // std::vector< ob::State* > &states = path_->getState();
    // ob::State *state;

    // // moveit_msgs::RobotTrajectory
    // //vector<float64> 

    for(unsigned int i=0; i<path_->getStateCount(); ++i){

        const ob::State* state = path_->getState(i);
        moveit_msgs::msg::RobotTrajectory robot_trajectory;

        auto joint1 = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        auto joint2 = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        auto joint3 = state->as<ob::RealVectorStateSpace::StateType>()->values[2];
        auto joint4 = state->as<ob::RealVectorStateSpace::StateType>()->values[3];
        auto joint5 = state->as<ob::RealVectorStateSpace::StateType>()->values[4];
        auto joint6 = state->as<ob::RealVectorStateSpace::StateType>()->values[5];

        std::cout<< "##########test3######"<<std::endl;

        // jakis problem z przypisaniem wartości joint stateow do debugu
        robot_trajectory.joint_trajectory.points.back().positions.push_back(joint1);
        std::cout<< "##########test4######"<<std::endl;
        display_trajectory.trajectory.push_back(robot_trajectory);
        

    }

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
    robot_state->setToDefaultValues();

    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("ur_manipulator");
    // planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    RCLCPP_INFO(logger, "##########test######");

    // planowanie sciezki
    auto path=plan();
     RCLCPP_INFO(logger, "##########test2######");
  
    //przeksztalcenie jej na wiadomosc do wizualizacji
    auto disTraj=extractPath(path);

 





    // for(unsigned int i=0; i<path_->getStateCount(); ++i){

    //     const ob::State* state2 = path_->getState(i);

    //     const auto *joint1 = state2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        // const auto joint2 = state2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
        // const auto joint3 =  state2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);
        // const auto joint4 =  state2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(3);
        // const auto joint5 =  state2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(4); ,joint2->values[0],joint3->values[0],joint4->values[0],joint5->values[0],joint6->values[0]
        // const auto joint6 = state2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(5); , joint2: %lf , joint3: %lf , joint4: %lf, joint5: %lf, joint6 %lf

        // std::cout<<"test"<<std::endl;
        // std::cout<<(double)joint1->values[0]<<std::endl;

        // RCLCPP_INFO(logger,"Path %d , joint1: %f ",i,joint1->values[0]);


    // }
   
   // wizualizacja
    moveit_visual_tools::MoveItVisualTools visual_tools{node,"shoulder_pan_joint",rviz_visual_tools::RVIZ_MARKER_TOPIC,
    robot_model};
    visual_tools.loadRobotStatePub("/display_planned_path");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();

    visual_tools.loadRemoteControl();

    visual_tools.trigger();
 




    rclcpp::shutdown();
    spinner.join();




    return 0;
}