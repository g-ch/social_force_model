#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tuple>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include "social_force_model.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

/*********
 * TODO: The static obstacle force looks weird. Check if it is correct.
 * ******/



Eigen::Vector2d goal;
std::vector<double> robot_state;

geometry_msgs::PoseStamped object_one_pose;
geometry_msgs::PoseStamped object_two_pose;

bool goal_received = false;
bool robot_state_received = false;
bool object_one_pose_received = false;
bool object_two_pose_received = false;


// Convert a double value to RGB color
std::tuple<int, int, int> doubleToRGB(double value, double min = 0.0, double max = 10.0) {
    // Normalize value
    value = (value - min) / (max - min);

    int red, green, blue;
    double min_to_mid = 0.5 * (max - min);

    if (value < min_to_mid) {
        // Transition from green to yellow
        red = static_cast<int>(2 * value * 255); // Increasing red
        green = 255; // Full green
    } else {
        // Transition from yellow to red
        red = 255; // Full red
        green = static_cast<int>((1.0 - value) * 2 * 255); // Decreasing green
    }

    return std::make_tuple(red, green, blue);
}


void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_state.clear();
    robot_state.push_back(msg->pose.pose.position.x); //x
    robot_state.push_back(msg->pose.pose.position.y); //y
    robot_state.push_back(0.0); //theta
    robot_state.push_back(msg->twist.twist.linear.x); //vx
    robot_state.push_back(msg->twist.twist.linear.y); //vy
    robot_state.push_back(0.5); //radius
    robot_state_received = true;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal << msg->pose.position.x, msg->pose.position.y;
    goal_received = true;
}


void objectOnePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    object_one_pose_received = true;
    object_one_pose = *msg;
}

void objectTwoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    object_two_pose_received = true;
    object_two_pose = *msg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "social_force_model_node");

    // Get Yaml file path
    std::string yaml_file_path = "/home/clarence/ros_ws/dingo_ws/src/social_force_model/config/cfg.yaml";

    // Read Yaml file
    YAML::Node yaml_file = YAML::LoadFile(yaml_file_path);

    double force_factor_goal = yaml_file["force_factor_goal"].as<double>();
    double force_factor_social = yaml_file["force_factor_social"].as<double>();
    double force_factor_obstacle = yaml_file["force_factor_obstacle"].as<double>();

    double visualization_resolusion = yaml_file["visualization_resolusion"].as<double>();
    double visualization_range_x = yaml_file["visualization_range_x"].as<double>();
    double visualization_range_y = yaml_file["visualization_range_y"].as<double>();

    std::string robot_odom_topic = yaml_file["robot_odom_topic"].as<std::string>();
    std::string goal_topic = yaml_file["goal_topic"].as<std::string>();
    std::string object_one_pose_topic = yaml_file["object_one_pose_topic"].as<std::string>();
    std::string object_two_pose_topic = yaml_file["object_two_pose_topic"].as<std::string>();


    // Define publishers
    ros::NodeHandle nh;
    ros::Publisher pub_total_force = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("social_force_model/total_force", 1);
    ros::Publisher pub_goal_force = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("social_force_model/goal_force", 1);
    ros::Publisher pub_static_obstacle_force = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("social_force_model/static_obstacle_force", 1);
    ros::Publisher pub_social_force = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("social_force_model/social_force", 1);

    ros::Subscriber robot_odom_sub = nh.subscribe(robot_odom_topic, 1, robotOdomCallback);
    ros::Subscriber goal_sub = nh.subscribe(goal_topic, 1, goalCallback);
    ros::Subscriber object_one_pose_sub = nh.subscribe(object_one_pose_topic, 1, objectOnePoseCallback);
    ros::Subscriber object_two_pose_sub = nh.subscribe(object_two_pose_topic, 1, objectTwoPoseCallback);
    
    // Define social force model
    SocialForceModel sfm;

    sfm.setForceFactorDesired(force_factor_goal);
    sfm.setForceFactorObstacle(force_factor_obstacle);
    sfm.setForceFactorSocial(force_factor_social);

    double map_range_x = visualization_range_x;
    double map_range_y = visualization_range_y;
    double checking_interval = visualization_resolusion;


    std::vector<std::vector<double>> static_obj_states; /// Not used for now.
    std::vector<std::vector<double>> dynamic_obj_states;

    pcl::PointCloud<pcl::PointXYZRGB> total_force_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> goal_force_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> static_obstacle_force_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> social_force_cloud;

    // Wait for robot state and goal state
    ros::Rate wait_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();

        if(robot_state_received && goal_received)
            break;

        wait_rate.sleep();
    }


    // Get into the main loop
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        std::cout << "Calculating forces..." << std::endl;
        double biggest_force = 0.0;
        
        static_obj_states.clear();
        dynamic_obj_states.clear();

        // Add dynamic objects
        if(object_one_pose_received)
        {
            std::vector<double> object_one_state;
            object_one_state.push_back(object_one_pose.pose.position.x); //x
            object_one_state.push_back(object_one_pose.pose.position.y); //y
            object_one_state.push_back(0.0); //theta
            object_one_state.push_back(0.001); //vx
            object_one_state.push_back(0.0); //vy
            object_one_state.push_back(0.4); //radius

            dynamic_obj_states.push_back(object_one_state);
        }

        if(object_two_pose_received)
        {
            std::vector<double> object_two_state;
            object_two_state.push_back(object_two_pose.pose.position.x); //x
            object_two_state.push_back(object_two_pose.pose.position.y); //y
            object_two_state.push_back(0.0); //theta
            object_two_state.push_back(0.001); //vx
            object_two_state.push_back(0.0); //vy
            object_two_state.push_back(0.4); //radius

            dynamic_obj_states.push_back(object_two_state);
        }


        for(double x = -map_range_x; x < map_range_x; x += checking_interval)
        {
            for(double y = -map_range_y; y < map_range_y; y += checking_interval)
            {   
                // Calculate total force
                Eigen::Vector2d goal_force, static_obstacle_force, social_force;
                Eigen::Vector2d total_force = sfm.sfmStep(robot_state, static_obj_states, dynamic_obj_states, goal, goal_force, static_obstacle_force, social_force);

                if(total_force.norm() > biggest_force)
                    biggest_force = total_force.norm();

                // Create point cloud for visualization
                pcl::PointXYZRGB goal_force_point, static_obstacle_force_point, social_force_point, total_force_point;
                goal_force_point.x = static_obstacle_force_point.x = social_force_point.x = total_force_point.x = x;
                goal_force_point.y = static_obstacle_force_point.y = social_force_point.y = total_force_point.y = y;
                goal_force_point.z = static_obstacle_force_point.z = social_force_point.z = total_force_point.z = 0.0;

                std::tuple<int, int, int> goal_force_color = doubleToRGB(goal_force.norm(), 0.0, 1.0);
                std::tuple<int, int, int> static_obstacle_force_color = doubleToRGB(static_obstacle_force.norm());
                std::tuple<int, int, int> social_force_color = doubleToRGB(social_force.norm());
                std::tuple<int, int, int> total_force_color = doubleToRGB(total_force.norm());

                goal_force_point.r = std::get<0>(goal_force_color);
                goal_force_point.g = std::get<1>(goal_force_color);
                goal_force_point.b = std::get<2>(goal_force_color);

                static_obstacle_force_point.r = std::get<0>(static_obstacle_force_color);
                static_obstacle_force_point.g = std::get<1>(static_obstacle_force_color);
                static_obstacle_force_point.b = std::get<2>(static_obstacle_force_color);

                social_force_point.r = std::get<0>(social_force_color);
                social_force_point.g = std::get<1>(social_force_color);
                social_force_point.b = std::get<2>(social_force_color);

                total_force_point.r = std::get<0>(total_force_color);
                total_force_point.g = std::get<1>(total_force_color);
                total_force_point.b = std::get<2>(total_force_color);

                goal_force_cloud.push_back(goal_force_point);
                static_obstacle_force_cloud.push_back(static_obstacle_force_point);
                social_force_cloud.push_back(social_force_point);
                total_force_cloud.push_back(total_force_point);
            }
        }

        std::cout << "Biggest force: " << biggest_force << std::endl;

        // Publish total force, goal force, static obstacle force, social force
        total_force_cloud.header.frame_id = "map";
        goal_force_cloud.header.frame_id = "map";
        static_obstacle_force_cloud.header.frame_id = "map";
        social_force_cloud.header.frame_id = "map";

        pub_total_force.publish(total_force_cloud);
        pub_goal_force.publish(goal_force_cloud);
        pub_static_obstacle_force.publish(static_obstacle_force_cloud);
        pub_social_force.publish(social_force_cloud);
    }

    return 0;
}