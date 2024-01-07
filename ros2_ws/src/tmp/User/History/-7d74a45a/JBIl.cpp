#include <chrono>
#include <builtin_interfaces/msg/time.hpp>
#include <iomanip>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>  
#include <ompl/config.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include </opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>


 namespace ob = ompl::base;
 namespace og = ompl::geometric;
  
 bool isStateValid(const ob::State *state)
 {
     // cast the abstract state type to the type we expect
     const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

	// extract the first component of the state and cast it to what we expect
	const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

	// extract the second component of the state and cast it to what we expect
	const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

	// check validity of state defined by pos & rot
	double x = (*pos)[0];
	double y = (*pos)[1];
	double theta = rot->value;  // In SE2, the rotation is represented by a single angle (theta) in SO2.
	

    // Check if the position is within certain bounds
    if (x < 0 || x > 348 || y < 0 || y > 348 || theta<-3.14 || theta>3.14) {
        return false; // State is invalid due to bounds violation
    }
    YAML::Node config = YAML::LoadFile("map.yaml");
    double occt=config["occupied_thresh"].as<double>();
    //double frt=config["free_thresh"].as<double>();
    cv::Mat image = cv::imread("map.pgm", cv::IMREAD_GRAYSCALE);
    cv::Scalar intensity = image.at<uchar>(cv::Point(x, y));
    //Checks for obstacle
    if(intensity.val[0]<=occt){
        return false;
    }
     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
     return (const void*)rot != (const void*)pos;
 }
  
 void plan(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& path_publisher,const rclcpp::Node::SharedPtr& node)
 {
     // construct the state space we are planning in
	auto space(std::make_shared<ob::SE2StateSpace>());

	// set the bounds for the R^2 part of SE(2)
    int mapWidth=384;
    int mapHeight=384;
    YAML::Node config = YAML::LoadFile("map.yaml");
    // Extract parameters
    std::string image_path = config["image"].as<std::string>();
    double resolution = config["resolution"].as<double>();
    YAML::Node originNode = config["origin"];
    double originX = originNode[0].as<double>();
    double originY = originNode[1].as<double>();
    double mapWidthMeters = mapWidth * resolution;
    double mapHeightMeters = mapHeight * resolution;
    
    
	ob::RealVectorBounds bounds(2);
    bounds.setLow(0, originX);
    bounds.setHigh(0, originX + mapWidthMeters);
    bounds.setLow(1, originY);
    bounds.setHigh(1, originY + mapHeightMeters);
    space->setBounds(bounds);

	// construct an instance of space information from this state space
	auto si(std::make_shared<ob::SpaceInformation>(space));

	// set state validity checking for this space
	si->setStateValidityChecker(isStateValid);

	// create a random start state
	ob::ScopedState<> start(space);
	start.random();

	// create a random goal state
	ob::ScopedState<> goal(space);
	std::cout << goal;
	goal[0] = 0.5; // Set X-coordinate of the goal
	goal[1] = -0.5; // Set Y-coordinate of the goal
	// Set the orientation if applicable (e.g., for SE3 planning)
	goal[2] = 0;

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<og::RRTConnect>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        auto solution_path = pdef->getSolutionPath()->as<og::PathGeometric>();
        auto path_msg = std::make_unique<nav_msgs::msg::Path>();
        auto current_time = std::chrono::system_clock::now();

        //Convert std::chrono::system_clock::time_point to builtin_interfaces::msg::Time
        auto time_msg = builtin_interfaces::msg::Time();
        time_msg.sec = std::chrono::time_point_cast<std::chrono::seconds>(current_time).time_since_epoch().count();
        time_msg.nanosec = (current_time.time_since_epoch() % std::chrono::seconds(1)).count();

        path_msg->header.stamp = time_msg;
        path_msg->header.frame_id = "map"; // Set the frame ID as per your requirements

        // Extract states from the solution path and add them to the ROS 2 Path message
        for (std::size_t i = 0; i < solution_path->getStateCount(); ++i)
        {
            const ob::State* state = solution_path->getState(i);
            const auto* se2_state = state->as<ob::SE2StateSpace::StateType>();
            const auto* pos = se2_state->as<ob::RealVectorStateSpace::StateType>(0);
            const auto* rot = se2_state->as<ob::SO2StateSpace::StateType>(1);

            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = (*pos)[0];
            pose.pose.position.y = (*pos)[1];
            pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), rot->value));
            path_msg->poses.push_back(pose);
        }

        // Publish the path
        path_publisher->publish(std::move(path_msg));

    }
    else
        std::cout << "No solution found" << std::endl;

 }
  
 void planWithSimpleSetup(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& path_publisher,const rclcpp::Node::SharedPtr& node)
 {
     // construct the state space we are planning in
     auto space(std::make_shared<ob::SE2StateSpace>());
  
     // set the bounds for the R^3 part of SE(3)
     ob::RealVectorBounds bounds(3);
     bounds.setLow(-1);
     bounds.setHigh(1);
  
     space->setBounds(bounds);
  
     // define a simple setup class
     og::SimpleSetup ss(space);
  
     // set state validity checking for this space
     ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
  
     // create a random start state
     ob::ScopedState<> start(space);
     start.random();
  
     // create a random goal state
     ob::ScopedState<> goal(space);
     goal.random();
     std::cout<<"GOAL:"<<goal;
  
     // set the start and goal states
     ss.setStartAndGoalStates(start, goal);
  
     // this call is optional, but we put it in to get more output information
     ss.setup();
     ss.print();
  
     // attempt to solve the problem within one second of planning time
     ob::PlannerStatus solved = ss.solve(1.0);
  
     if (solved)
     {
         std::cout << "Found solution:" << std::endl;
         // print the path to screen
         ss.simplifySolution();
         og::PathGeometric& solution_path=ss.getSolutionPath();
         auto path_msg = std::make_unique<nav_msgs::msg::Path>();
         path_msg->header.stamp = node->get_clock()->now();
         path_msg->header.frame_id = "map"; // Set the frame ID as per your requirements

        // Extract states from the solution path and add them to the ROS 2 Path message
         for (std::size_t i = 0; i < solution_path->getStateCount(); ++i)
         {
            const ob::State* state = solution_path->getState(i);
            const auto* se2_state = state->as<ob::SE2StateSpace::StateType>();
            const auto* pos = se2_state->as<ob::RealVectorStateSpace::StateType>(0);
            const auto* rot = se2_state->as<ob::SO2StateSpace::StateType>(1);

            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = (*pos)[0];
            pose.pose.position.y = (*pos)[1];
            pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), rot->value));
            path_msg->poses.push_back(pose);
        }

        // Pubconst rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& path_publisherlish the path
        path_publisher->publish(std::move(path_msg));
     }
     else
         std::cout << "No solution found" << std::endl;
 }
  
 int main(int argc, char** argv)
 {
    /*rclcpp::init(argc, argv);
    auto node=std::make_shared<rclcpp::Node>("plan_publisher");
    auto pather=node->create_publisher<nav_msgs::msg::Path>("/plannedpath",10);
    
    
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
  
     plan(pather,node);
     std::cout<<"End"<<std::endl;
  
     std::cout << std::endl << std::endl;
  
     planWithSimpleSetup(pather,node);*/
  
     return 0;
 }
