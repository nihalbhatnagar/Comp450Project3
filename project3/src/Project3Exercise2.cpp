///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <fstream>
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
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(3);


 
    space->setBounds(bounds);

    og::SimpleSetup ss(space);

    ss.setStateValidityChecker([&obstacles](const ob::State *state) { 
        
        bool st = isValidStatePoint(state, obstacles);
        
        const ompl::base::RealVectorStateSpace::StateType* R2State = state->as<ompl::base::RealVectorStateSpace::StateType>();
        double x = R2State->values[0];
        double y = R2State->values[1];
        std::cout << x << " " << y << std::endl; 
        return st;
        });

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    start[0] = 2;
    start[1] = -1;

    // define goal state
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
    goal[0] = -1;
    goal[1] = 2;

    ss.setStartAndGoalStates(start, goal);

    // RTP planner(space)
    auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    ob::PlannerStatus solved = ss.solve(1.0);


    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        //ss.simplifySolution();
        //ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
        
        std::ofstream file("src/pointPath.txt");

        ss.getSolutionPath().printAsMatrix(file);

        file.close();
    }
}

void planBox(const std::vector<Rectangle> &obstacles)
{
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    // TODO: Use your implementation of RTP to plan for a rotating square robot.
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());
    double sideLength = .5;
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(3);
 
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([&obstacles, sideLength](const ob::State *state) { return isValidStateSquare(state, sideLength, obstacles); });

    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
    start->setX(-1);
    start->setY(3);
    start->setYaw(0);

    // define goal state
    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
    goal->setX(3);
    goal->setY(-1);
    goal->setYaw(0);
    ss.setStartAndGoalStates(start, goal);

    // RTP planner(space)
    auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    ob::PlannerStatus solved = ss.solve(5.0);


    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        //ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
        
        std::ofstream file("src/boxPath.txt");

        ss.getSolutionPath().printAsMatrix(file);

        file.close();
    }
}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    // Rectangle obstacle1;
    // obstacle1.x = .01;
    // obstacle1.y = .01;
    // obstacle1.height = .4;
    // obstacle1.width = .4;
    // obstacles.push_back(obstacle1);

    // Rectangle obstacle2;
    // obstacle2.x = 13.0;
    // obstacle2.y = 8.0;
    // obstacle2.height = 20.0;
    // obstacle2.width = 9.0;
    // obstacles.push_back(obstacle2);
    Rectangle obstacle1;
    obstacle1.x = 1.0;
    obstacle1.y = 2.0;
    obstacle1.height = 1.0;
    obstacle1.width = 1.5;
    obstacles.push_back(obstacle1);

    Rectangle obstacle2;
    obstacle2.x = 0;
    obstacle2.y = 1;
    obstacle2.height = .5;
    obstacle2.width = .5;
    obstacles.push_back(obstacle2);

    Rectangle obstacle3;
    obstacle3.x = 1;
    obstacle3.y = .5;
    obstacle3.height = .5;
    obstacle3.width = .3;
    obstacles.push_back(obstacle3);

    Rectangle obstacle4;
    obstacle4.x = -1.5;
    obstacle4.y = 1.0;
    obstacle4.height = .75;
    obstacle4.width = 1;
    obstacles.push_back(obstacle4);

    Rectangle obstacle5;
    obstacle5.x = .8;
    obstacle5.y = -1.5;
    obstacle5.height = .5;
    obstacle5.width = 1;
    obstacles.push_back(obstacle5);

    // Rectangle obstacle6;
    // obstacle6.x = 0;
    // obstacle6.y = 0;
    // obstacle6.height = 20.0;
    // obstacle6.width = 20.0;
    // obstacles.push_back(obstacle6);

    // // Rectangle obstacle6;
    // // obstacle6.x = 5;
    // // obstacle6.y = 5;
    // // obstacle6.height = 30;
    // // obstacle6.width = 30;
    // // obstacles.push_back(obstacle6);

}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    Rectangle obstacle1;
    obstacle1.x = -1.0;
    obstacle1.y = 1.0;
    obstacle1.height = .5;
    obstacle1.width = .5;
    obstacles.push_back(obstacle1);

    Rectangle obstacle2;
    obstacle2.x = -.25;
    obstacle2.y = -.25;
    obstacle2.height = .2;
    obstacle2.width = .2;
    obstacles.push_back(obstacle2);

    Rectangle obstacle3;
    obstacle3.x = 0;
    obstacle3.y = 0;
    obstacle3.height = .3;
    obstacle3.width = .3;
    obstacles.push_back(obstacle3);

    Rectangle obstacle4;
    obstacle4.x = .4;
    obstacle4.y = .4;
    obstacle4.height = .35;
    obstacle4.width = .35;
    obstacles.push_back(obstacle4);

    Rectangle obstacle5;
    obstacle5.x = .9;
    obstacle5.y = .9;
    obstacle5.height = 1;
    obstacle5.width = 1;
    obstacles.push_back(obstacle5);
    Rectangle obstacle6;
    obstacle6.x = 2.5;
    obstacle6.y = 2.3;
    obstacle6.height = .2;
    obstacle6.width = .5;
    obstacles.push_back(obstacle6);

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
