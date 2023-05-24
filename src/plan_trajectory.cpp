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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;


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

    // const auto *coordX =
    //         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    // // get y coord of the robot
    // const auto *coordY =
    //         state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    
    // const auto *coordZ = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(2);

    // //! Comment this part of the code if you'd like to use occupancy grid
    // // define the obstacle
    // if (coordX->values[0]<5.1&&coordX->values[0]>5.0){
    //     if (coordY->values[0]<4.0&&coordY->values[0]>-5.0){

    //         if(coordZ->values[0]<10.0&& coordZ->values[0]>-5.0){

    //             return false;

    //         }
    //     }
    // }

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




void planWithSimpleSetup()
{
    // construct the state space we are planning in
    auto si(std::make_shared<ob::SpaceInformation>(space));
    // ob::RealVectorBounds bounds(3);
    // bounds.setLow(-1);
    // bounds.setHigh(1);

    // space->setBounds(bounds);
    // og::SimpleSetup ss(space);

    si->setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
    si->setStateValidityCheckingResolution(0.01);

    // ob::ScopedState<> start(space);
    // start.random();

    // ob::ScopedState<> goal(space);
    // goal.random();


    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(*start.get(), *goal.get());


   
    // ss.setStartAndGoalStates(start, goal);

    auto planner(std::make_shared<og::RRTConnect>(si));
    // configure the planner
    planner->setRange(maxStepLength);// max step length
    planner->setProblemDefinition(pdef);
    planner->setup();

    ob::PlannerStatus solved =  planner->ob::Planner::solve(1.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        // ss.simplifySolution();
        // ss.getSolutionPath().print(std::cout);
    }
    else{
        std::cout << "Problem has occured:" << std::endl;
    }



}


void plan(){

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


    // using moveit::planning_interface::MoveGroupInterface;
    // auto move_group = MoveGroupInterface(node, "ur_manipulator");
    // RCLCPP_INFO(logger, "##########test######");


    
    // RCLCPP_INFO(logger, "getDefaultPlanningPipelineId: %s", move_group.getDefaultPlanningPipelineId().c_str());
    // RCLCPP_INFO(logger,"Reference frame: %s", move_group.getPlanningFrame().c_str());
    // RCLCPP_INFO(logger,"Reference frame: %s", move_group.getEndEffectorLink().c_str());

    plan();
   
    // configure();

    // og::SimpleSetup ss(move_group.getRobotModel());
    // planWithSimpleSetup();
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