// /*********************************************************************
//  * Rice University Software Distribution License
//  *
//  * Copyright (c) 2010, Rice University
//  * All Rights Reserved.
//  *
//  * For a full description see the file named LICENSE.
//  *
//  *********************************************************************/

// /* Author: Ioan Sucan */


#include "RTP.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

using namespace ompl;

// int main()
// {
//     // plan in SE2
//     app::SE2RigidBodyPlanning setup;

//     // load the robot and the environment
//     std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car1_planar_robot.dae";
//     std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Maze_planar_env.dae";
//     setup.setRobotMesh(robot_fname);
//     setup.setEnvironmentMesh(env_fname);

//     // define starting state
//     base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
//     start->setX(0.0);
//     start->setY(0.0);

//     // define goal state
//     base::ScopedState<base::SE2StateSpace> goal(start);
//     goal->setX(26.0);
//     goal->setY(0.0);
    
//     // set the start & goal states
//     setup.setStartAndGoalStates(start, goal);
//     //set the planner

//     setup.setPlanner();
//     // attempt to solve the problem, and print it to screen if a solution is found
//     if (setup.solve())
//         setup.getSolutionPath().print(std::cout);

//     return 0;

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state){

}

void planWithSimpleSetup()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
 
    space->setBounds(bounds);

    // og::SimpleSetup ss(space);
    // ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

    ob::ScopedState<> start(space);
    start.random();

    ob::ScopedState<> goal(space);
    goal.random();

    ss.setStartAndGoalStates(start, goal);

    RTP planner(space)
    ss.setPlanner(planner)

    ob::PlannerStatus solved = ss.solve(1.0);


    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
}