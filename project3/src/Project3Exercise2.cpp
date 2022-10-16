///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

// The collision checker routines
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/State.h>

void planPoint(const std::vector<Rectangle> &obstacles)
{
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
 
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([&obstacles](const ob::State *state) { return isValidStatePoint(state, obstacles); });

    ob::ScopedState<> start(space);
    start.random();

    ob::ScopedState<> goal(space);
    goal.random();

    ss.setStartAndGoalStates(start, goal);

    // RTP planner(space)
    auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    ob::PlannerStatus solved = ss.solve(1.0);


    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
}

void planBox(const std::vector<Rectangle> &obstacles)
{
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    // TODO: Use your implementation of RTP to plan for a rotating square robot.
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());
    double sideLength = 3;
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
 
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([&obstacles, sideLength](const ob::State *state) { return isValidStateSquare(state, sideLength, obstacles); });

    ob::ScopedState<> start(space);
    start.random();

    ob::ScopedState<> goal(space);
    goal.random();

    ss.setStartAndGoalStates(start, goal);

    // RTP planner(space)
    auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    ob::PlannerStatus solved = ss.solve(1.0);


    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    Rectangle obstacle1;
    obstacle1.x = 1.0;
    obstacle1.y = 2.0;
    obstacle1.height = 5.0;
    obstacle1.width = 15.0;
    obstacles.push_back(obstacle1);

    Rectangle obstacle2;
    obstacle2.x = 13.0;
    obstacle2.y = 8.0;
    obstacle2.height = 20.0;
    obstacle2.width = 9.0;
    obstacles.push_back(obstacle2);
}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your second environment.
    Rectangle obstacle1;
    obstacle1.x = 1.0;
    obstacle1.y = 2.0;
    obstacle1.height = 5.0;
    obstacle1.width = 15.0;
    obstacles.push_back(obstacle1);

    Rectangle obstacle2;
    obstacle2.x = 13.0;
    obstacle2.y = 8.0;
    obstacle2.height = 20.0;
    obstacle2.width = 9.0;
    obstacles.push_back(obstacle2);
}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) TODO" << std::endl;
        std::cout << " (2) TODO" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
