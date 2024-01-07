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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
 namespace ob = ompl::base;
 namespace og = ompl::geometric;


cv::Mat image;
double occt;
double resolution,originX,originY;
 bool isStateValid(const ob::State *state)
 {
 	
        const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
	const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
	//const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);
	double x = (*pos)[0];
	double y = (*pos)[1];
	
	//double theta = rot->value;  
    	//double map_width = 348 * 0.05; // Convert map width from pixels to meters
    	//double map_height = 348 * 0.05; // Convert map height from pixels to meters

    // Use these variables in the if statement:
    int cox = (int)((x-originX) /resolution);
    int coy = (int)(y-originY/resolution);
    try{
    if (image.empty())
    {
        std::cerr << "Error: Image not loaded." << std::endl;
        return false;
    }
    std::cout<<"OriginX:"<<originX<<"  Coordinates:"<<"cox:"<<cox<<"  coy:"<<coy<<"  X:"<<x<<"  Y:"<<y<<std::endl;
    
    cv::Scalar intensity = image.at<uchar>(cv::Point(coy, cox));
    double inten=(255-intensity.val[0])/255.0;
    std::cout<<"Here  "<<occt<<std::endl;
    
    if(inten>occt){
        return false;
    }
    }
        catch (...)  {
        std::cout << "Default Exception\n";
    }
     return true;
 }

 void plan(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& path_publisher)
 {
	auto space(std::make_shared<ob::SE2StateSpace>());
    int mapWidth=125;
    int mapHeight=116;
    YAML::Node config = YAML::LoadFile("/root/ros2_ws/src/maps/map.yaml");
    std::string image_path = config["image"].as<std::string>();
    resolution = config["resolution"].as<double>();
    YAML::Node originNode = config["origin"];
    originX = originNode[0].as<double>();
    std::cout<<"OriginX:"<<originX<<std::endl;
    originY = originNode[1].as<double>();
    double mapWidthMeters = mapWidth * resolution;
    double mapHeightMeters = mapHeight * resolution;
	ob::RealVectorBounds bounds(2);
    bounds.setLow(0, (originX));
    bounds.setHigh(0, (originX) + mapWidthMeters);
    bounds.setLow(1, (originY));
    bounds.setHigh(1, (originY) + mapHeightMeters);
    bounds.setLow(2, -3.14);
    bounds.setHigh(2, 3.14);
    space->setBounds(bounds);
    
    
	auto si(std::make_shared<ob::SpaceInformation>(space));
	si->setStateValidityChecker(isStateValid);
	ob::ScopedState<> start(space);
	start[0] =0.0647422; 
	start[1] =-0.158908; 
	start[2] = 0;
	ob::ScopedState<> goal(space);
	std::cout << goal;
	goal[0] = 2.9905;
	goal[1] = -0.0174552;
	goal[2] = 0;
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);
    auto planner(std::make_shared<og::RRTConnect>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();
    si->printSettings(std::cout);
    pdef->print(std::cout);
    ob::PlannerStatus solved = planner->ob::Planner::solve(100.0);

    if (solved)
    {
        auto solution_path = pdef->getSolutionPath()->as<og::PathGeometric>();
        auto path_msg = std::make_unique<nav_msgs::msg::Path>();
        auto current_time = std::chrono::system_clock::now();
        auto time_msg = builtin_interfaces::msg::Time();
        time_msg.sec = std::chrono::time_point_cast<std::chrono::seconds>(current_time).time_since_epoch().count();
        time_msg.nanosec = (current_time.time_since_epoch() % std::chrono::seconds(1)).count();
        path_msg->header.stamp = time_msg;
        path_msg->header.frame_id = "map"; 
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
        std::cout<<"SOLVED!"<<std::endl;
        solution_path->printAsMatrix(std::cout);
        path_publisher->publish(std::move(path_msg));
        std::cout<<"PUBLISHED!"<<std::endl;

    }
    else{
        std::cout << "No solution found" << std::endl;

        }
 }
 int main(int argc, char** argv)
 {
    std::cout<<"I'm here";
    rclcpp::init(argc, argv);
    auto node=std::make_shared<rclcpp::Node>("plan_publisher");
    auto pather=node->create_publisher<nav_msgs::msg::Path>("/planned_path",10);
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    YAML::Node config = YAML::LoadFile("/root/ros2_ws/src/maps/map.yaml");
    occt=config["occupied_thresh"].as<double>();
    image = cv::imread("/root/ros2_ws/src/maps/map.pgm", cv::IMREAD_GRAYSCALE); 
    /*cv::Mat dst;      //Mat object for output image file
    cv::Point2f pt(image.cols/2., image.rows/2.);          //point from where to rotate    
    cv::Mat r = cv::getRotationMatrix2D(pt, -270, 1.0);      //Mat object for storing after rotation
    cv::warpAffine(image, dst, r, cv::Size(image.cols, image.rows));  ///applie an affine transforation to image.
    image=dst;         //returning Mat object for output image file
    cv::imshow("Window Name", image); 
  
    // Wait for any keystroke 
    cv::waitKey(0); */
    plan(pather);
    
    rclcpp::spin(node);
    return 0;
 }
