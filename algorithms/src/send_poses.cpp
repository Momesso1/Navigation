#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <map>
#include <stack>
#include <unordered_map>
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class SendPoses : public rclcpp::Node {
private:

    // Publishers.
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;

    // Subscriptions.
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Timers.
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr parameterTimer;
    
    geometry_msgs::msg::Point drone_pos_;
    std::unordered_map<std::string, geometry_msgs::msg::Pose> detected_objects;
    std::vector<geometry_msgs::msg::Pose> destinations_;
    std::vector<geometry_msgs::msg::Pose> dockingDestinations_;

    bool docking_;

    // Função para carregar as localizações do arquivo YAML
    void load_locations_from_yaml(const std::string &file_path)
    {
        try 
        {
            YAML::Node config = YAML::LoadFile(file_path);
            for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) 
            {
                std::vector<double> coords = it->second.as<std::vector<double>>();
                if (coords.size() >= 3) 
                {
                    
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = coords[0];
                    pose.position.y = coords[1];
                    pose.position.z = coords[2];
                    pose.orientation.x = 0.0;
                    pose.orientation.y = 0.0;
                    pose.orientation.z = 0.0;
                    pose.orientation.w = 1.0;
                    destinations_.push_back(pose);
                }
            }
        } 
        catch (const YAML::Exception &e) 
        {
        std::cerr << "Erro ao carregar o arquivo YAML: " << e.what() << std::endl;
        }
    }

    /*
        PUBLISHERS.
    */
    
    void publisher_poses()
    {   
        geometry_msgs::msg::PoseArray message;
        message.header.stamp = this->now();
        message.header.frame_id = "map";

       
        if(docking_ == false) 
        {

            for (const auto &vertex : destinations_) 
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = vertex.position.x;
                pose.position.y = vertex.position.y;
                pose.position.z = vertex.position.z;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                pose.orientation.w = 1.0; 
                message.poses.push_back(pose);
            }

        }
        else if (docking_ == true) 
        {

            for (const auto &vertex : dockingDestinations_) 
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = vertex.position.x;
                pose.position.y = vertex.position.y;
                pose.position.z = vertex.position.z;
                pose.orientation.x = vertex.orientation.x;
                pose.orientation.y = vertex.orientation.y;
                pose.orientation.z = vertex.orientation.z;
                pose.orientation.w = vertex.orientation.w; 
                message.poses.push_back(pose);
            }

        }


        publisher_->publish(message);
    }

    /*
        CALLBACKS.
    */

    void docking_positions(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        dockingDestinations_.clear();

        for (const auto &pose : msg->poses) {
        geometry_msgs::msg::Pose vertex; // Correto: declaramos uma variável do tipo Pose
        vertex.position.x = pose.position.x;
        vertex.position.y = pose.position.y;
        vertex.position.z = pose.position.z;

        vertex.orientation.x = pose.orientation.x;
        vertex.orientation.y = pose.orientation.y;
        vertex.orientation.z = pose.orientation.z;
        vertex.orientation.w = pose.orientation.w;

        dockingDestinations_.push_back(vertex);
        }
    }

   

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        drone_pos_ = msg->pose.pose.position; // Atualiza a posição do drone
    }

    void check_parameters()
    {
        auto new_docking = this->get_parameter("docking").get_parameter_value().get<bool>();

        if(new_docking != docking_) 
        {
            docking_ = new_docking;
            if(docking_ == true) 
            {
                RCLCPP_INFO(this->get_logger(), "The robot will dock now.");
            } 
            else 
            {
                RCLCPP_INFO(this->get_logger(), "The robot will not dock now");
            }
        }
    }


public:
  SendPoses()
  : Node("send_poses")
  {
    this->declare_parameter<bool>("docking", false);
    docking_ = this->get_parameter("docking").get_parameter_value().get<bool>();


    

    parameterTimer = this->create_wall_timer(1s, std::bind(&SendPoses::check_parameters, this));

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/destinations", 10);
    timer_ = this->create_wall_timer(1ms, std::bind(&SendPoses::publisher_poses, this));

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/docking_positions", 10, std::bind(&SendPoses::docking_positions, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/rtabmap/odom", 10, std::bind(&SendPoses::odom_callback, this, std::placeholders::_1));

    const std::string yaml_file_path = "/home/momesso/navigation/src/algorithms/config/locations.yaml";
    load_locations_from_yaml(yaml_file_path);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SendPoses>());
  rclcpp::shutdown();
  return 0;
}