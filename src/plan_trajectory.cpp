#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl-1.6/ompl/geometric/SimpleSetup.h>
#include <ompl-1.6/ompl/geometric/planners/rrt/RRTstar.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematic_constraints/utils.h>


#include <ompl-1.6/ompl/base/spaces/SE3StateSpace.h>
#include <ompl-1.6/ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl-1.6/ompl/base/spaces/RealVectorBounds.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state){

    return false;
}



// void plan()
// {    // construct the state space we are planning in
//     auto space(std::make_shared<ob::SE3StateSpace>());

//     ob::RealVectorBounds bounds(3);
//     bounds.setLow(-1);
//     bounds.setHigh(1);
 
//     space->setBounds(bounds);
//     auto si(std::make_shared<ob::SpaceInformation>(space));
//     si->setStateValidityChecker(isStateValid);

//     ob::ScopedState<> start(space);
//     start.random();
//     ob::ScopedState<> goal(space);
//     goal.random();
//     auto pdef(std::make_shared<ob::ProblemDefinition>(si));
//     pdef->setStartAndGoalStates(start, goal);
//     auto planner(std::make_shared<og::RRTConnect>(si));
//     planner->setProblemDefinition(pdef);
//     planner->setup();
//     ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

//     if (solved)
//     {
//         // get the goal representation from the problem definition (not the same as the goal state)
//         // and inquire about the found path
//         ob::PathPtr path = pdef->getSolutionPath();
//         std::cout << "Found solution:" << std::endl;
 
//         // print the path to screen
//         path->print(std::cout);
//     }

    
// }


void planWithSimpleSetup()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);
    og::SimpleSetup ss(space);

    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

    ob::ScopedState<> start(space);
    start.random();

    ob::ScopedState<> goal(space);
    goal.random();

    ss.setStartAndGoalStates(start, goal);

    ob::PlannerStatus solved = ss.solve(1.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
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


    // Next step goes here
    // Create the MoveIt MoveGroup Interface

    RCLCPP_INFO(logger, "##########test######");


    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, "ur_manipulator");
    RCLCPP_INFO(logger, "##########test######");


    
    RCLCPP_INFO(logger, "getDefaultPlanningPipelineId: %s", move_group.getDefaultPlanningPipelineId().c_str());
    RCLCPP_INFO(logger,"Reference frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(logger,"Reference frame: %s", move_group.getEndEffectorLink().c_str());

    // og::SimpleSetup ss(move_group.getRobotModel());
    planWithSimpleSetup();
    // Set up OMPL planner przypisuje robota do ompl plannera
    // ompl::geometric::SimpleSetup ss(move_group.getRobotModel());

    // Configure OMPL planner settings
    // ss.setStartAndGoalStates(start_state, goal_state);
    // // ... Set other planner parameters

    // // Configure the collision checker
    // ss.setStateValidityChecker([&](const ompl::base::State* state) {
    //     // Perform collision checking using MoveIt planning scene
    //     // ... Implement collision checking logic using MoveIt collision objects
    //     return isValid;
    // });

    // // Perform planning
    // ompl::base::PlannerStatus status = ss.solve(1.0);  // Plan for 5 seconds

    // if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
    // {
    //     // Retrieve and output the path
    //     ompl::geometric::PathGeometric path = ss.getSolutionPath();
    //     path.interpolate();
    //     for (std::size_t i = 0; i < path.getStateCount(); ++i)
    //     {
    //         const ompl::base::State* state = path.getState(i);
    //         // ... Convert OMPL state to MoveIt robot state and execute the trajectory
    //     }
    // }
    // else
    // {
    //     RCLCPP_INFO(node->get_logger(), "No solution found");
    // }

    // planWithSimpleSetup();

    // rclcpp::shutdown();
    // spinner.join();




    return 0;
}