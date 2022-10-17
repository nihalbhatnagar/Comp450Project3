///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>

// Your random tree planner
#include "RTP.h"

void benchmarkApartment()
{
    // plan in SE3
    ompl::app::SE3RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // define start state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(241.81);
    start->setY(106.15);
    start->setZ(36.46);
    start->rotation().setAxisAngle(0., 0., -1., 3.12413936107);

    // define goal state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(start);
    goal->setX(-31.19);
    goal->setY(-99.85);
    goal->setZ(36.46);
    goal->rotation().setAxisAngle(0., 0., -1., 3.12413936107);

    ompl::base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 295.77);
    bounds.setHigh(1, 168.26);
    bounds.setHigh(2, 90.39);
    bounds.setLow(0, -73.76);
    bounds.setLow(1, -179.59);
    bounds.setLow(2, -0.03);
    setup.getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // we call setup just so print() can show more information
    setup.setup();
    setup.print();

    // setup benchmarking
    std::string benchmark_name;
    double runtime_limit = 60.0;
    double memory_limit = 1000.0;
    int run_count = 30;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark b(setup, "Apartment Benchmark");

    auto rtp = std::make_shared<ompl::geometric::RTP>(setup.getSpaceInformation());
    rtp->setName("RTP");

    auto rrt = std::make_shared<ompl::geometric::RRT>(setup.getSpaceInformation());
    rrt->setName("RRT");

    auto prm = std::make_shared<ompl::geometric::PRM>(setup.getSpaceInformation());
    prm->setName("PRM");

    auto est = std::make_shared<ompl::geometric::EST>(setup.getSpaceInformation());
    est->setName("EST");

    b.addPlanner(rtp);
    b.addPlanner(rrt);
    b.addPlanner(prm);
    b.addPlanner(est);

    b.benchmark(request);
    b.saveResultsToFile();
}

void benchmarkHome()
{
    // plan in SE3
    ompl::app::SE3RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // define start state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(252.95);
    start->setY(-214.95);
    start->setZ(46.19);
    start->rotation().setAxisAngle(1., 0., 0., 0);

    // define goal state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(start);
    goal->setX(262.95);
    goal->setY(75.05);
    goal->setZ(46.19);
    goal->rotation().setAxisAngle(1., 0., 0., 0);

    ompl::base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 325.00);
    bounds.setHigh(1, 337.89);
    bounds.setHigh(2, 142.33);
    bounds.setLow(0, -383.80);
    bounds.setLow(1, -371.47);
    bounds.setLow(2, -0.20);
    setup.getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // we call setup just so print() can show more information
    setup.setup();
    setup.print();

    // setup benchmarking
    std::string benchmark_name;
    double runtime_limit = 90.0;
    double memory_limit = 10000.0;
    int run_count = 30;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark b(setup, "Home Benchmark");

    auto rtp = std::make_shared<ompl::geometric::RTP>(setup.getSpaceInformation());
    rtp->setName("RTP");

    auto rrt = std::make_shared<ompl::geometric::RRT>(setup.getSpaceInformation());
    rrt->setName("RRT");

    auto prm = std::make_shared<ompl::geometric::PRM>(setup.getSpaceInformation());
    prm->setName("PRM");

    auto est = std::make_shared<ompl::geometric::EST>(setup.getSpaceInformation());
    est->setName("EST");

    b.addPlanner(rtp);
    b.addPlanner(rrt);
    b.addPlanner(prm);
    b.addPlanner(est);

    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    int environment;

    do
    {
        std::cout << "Benchmark for: " << std::endl;
        std::cout << " (1) Apartment" << std::endl;
        std::cout << " (2) Home" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            benchmarkApartment();
            break;
        case 2:
            benchmarkHome();
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    return 0;
}
