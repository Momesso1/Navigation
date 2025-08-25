#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <cmath>
#include <map>
#include <unordered_map>
#include <optional>
#include <iostream>
#include <climits>
#include <iomanip>
#include <thread>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <cstring>
#include <filesystem>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
using namespace std::chrono_literals;

namespace std {
    template <>
    struct hash<std::pair<float, float>> {
        size_t operator()(const std::pair<float, float>& t) const {
            size_t h1 = hash<float>()(std::get<0>(t));
            size_t h2 = hash<float>()(std::get<1>(t));
            return h1 ^ (h2 << 1);  
        }
    };

    template <>
    struct hash<std::tuple<float, float, float>> {
        size_t operator()(const std::tuple<float, float, float>& t) const {
            size_t h1 = hash<float>()(std::get<0>(t));
            size_t h2 = hash<float>()(std::get<1>(t));
            size_t h3 = hash<float>()(std::get<2>(t));
            return h1 ^ (h2 << 1) ^ (h3 << 2); 
        }
    };
}


class GraphPublisher : public rclcpp::Node {

private:
    struct CloudMapPoint
    {
        bool verified;
    };

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_vertices_arbitrary;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher121_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_occupancy_grid;

    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription1_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_point_cloud_;
    rclcpp::TimerBase::SharedPtr timer_vertices_arbitrary;
    rclcpp::TimerBase::SharedPtr timer_vertices_arbitrary121_;
    rclcpp::TimerBase::SharedPtr timer_dynamic_map_;
    rclcpp::TimerBase::SharedPtr parameterTimer;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_fixed_vertices;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer_occupancy_grid;

   
    float resolution_;
    unsigned int width_;
    unsigned int height_;
    float poseX_ = 0.0;
    float poseY_ = 0.0;
    float poseZ_ = 0.0;
    float maxSecurityDistance_;
    float distanceToObstacle_;
    int decimals = 0;

    std::unordered_set<std::pair<float, float>> verticesArbitrary;
    std::unordered_map<std::pair<float, float>, bool> verticesCloudMap;

  

    inline float roundToMultiple(float value, float multiple, int decimals) {
        if (multiple == 0.0) return value; 
        
        float result = std::round(value / multiple) * multiple;
        float factor = std::pow(10.0, decimals);
        result = std::round(result * factor) / factor;
        
        return result;
    }

    int countDecimals(float number) 
    {
       
        float fractional = std::fabs(number - std::floor(number));
        int decimals = 0;
        const float epsilon = 1e-9; 
    
        while (fractional > epsilon && decimals < 20) 
        {
            fractional *= 10;
            fractional -= std::floor(fractional);
            decimals++;
        }
        return decimals;
    }
    

    void createGraphFromPointCloud() 
    {
        
        for(auto it = verticesCloudMap.begin(); it != verticesCloudMap.end(); it++)
        {
            if(it->second == false)
            {
                it->second = true;
                
                float toma = 0.0;
                int opa = 0;
              

                while(toma <= maxSecurityDistance_)
                {
                   
                    
                    for(int eita = 0; eita <= opa * 2; eita++)
                    {   
                        
                        std::pair<float, float> index10 = std::make_pair(roundToMultiple((std::get<0>(it->first) + toma) - (distanceToObstacle_ * eita), distanceToObstacle_, decimals), roundToMultiple((std::get<1>(it->first) + toma), distanceToObstacle_, decimals));
                        std::pair<float, float> index11 = std::make_pair(roundToMultiple((std::get<0>(it->first) + toma), distanceToObstacle_, decimals), roundToMultiple((std::get<1>(it->first) + toma) - (distanceToObstacle_ * eita), distanceToObstacle_, decimals));
                        
                        std::pair<float, float> index12 = std::make_pair(roundToMultiple((std::get<0>(it->first) - toma), distanceToObstacle_, decimals), roundToMultiple((std::get<1>(it->first) - toma) + (distanceToObstacle_ * eita), distanceToObstacle_, decimals));
                        std::pair<float, float> index13 = std::make_pair(roundToMultiple((std::get<0>(it->first) - toma) + (distanceToObstacle_ * eita), distanceToObstacle_, decimals), roundToMultiple((std::get<1>(it->first) - toma), distanceToObstacle_, decimals));
                        
                        
                        verticesArbitrary.insert(index10);
                        verticesArbitrary.insert(index11);
                        verticesArbitrary.insert(index12);
                        verticesArbitrary.insert(index13);
                        
                    }

                    opa++;
                
                   
                
    
                    toma += distanceToObstacle_;
                }
              
            }
            
        }

 
       
    }

    /*
    
        PUBLISHERS.

    */

    void publish_obstacles_vertices()
    {
        
        sensor_msgs::msg::PointCloud2 cloud_msg1;
        cloud_msg1.header.stamp = this->get_clock()->now();
        cloud_msg1.header.frame_id = "map";

        cloud_msg1.height = 1; 
        cloud_msg1.width = verticesArbitrary.size(); 
        cloud_msg1.is_dense = true;
        cloud_msg1.is_bigendian = false;
        cloud_msg1.point_step = 3 * sizeof(float); 
        cloud_msg1.row_step = cloud_msg1.point_step * cloud_msg1.width;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg1);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cloud_msg1.width);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg1, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg1, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg1, "z");
        for (const auto& vertex : verticesArbitrary) {
           
                *iter_x = std::get<0>(vertex);
                *iter_y = std::get<1>(vertex);
                *iter_z = 0.0;

                ++iter_x;
                ++iter_y;
                ++iter_z;
   
            
        }


         publisher_vertices_arbitrary->publish(cloud_msg1);
    
    }


    void occupancy_grid_callback()
    {
        auto grid_msg = nav_msgs::msg::OccupancyGrid();
        grid_msg.header.stamp = this->get_clock()->now();
        grid_msg.header.frame_id = "map";

        geometry_msgs::msg::Pose origin_;
        origin_.position.x = roundToMultiple(-20.0, distanceToObstacle_, decimals);
        origin_.position.y = roundToMultiple(-20.0, distanceToObstacle_, decimals);
        origin_.position.z = 0.0;
        origin_.orientation.x = 0.0;
        origin_.orientation.y = 0.0;
        origin_.orientation.z = 0.0;
        origin_.orientation.w = 1.0;
        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = width_;
        grid_msg.info.height = height_;
        grid_msg.info.origin = origin_;

        grid_msg.data.assign(width_ * height_, 0);

        for (const auto & pos : verticesArbitrary) 
        {
            int ix = static_cast<int>((std::get<0>(pos) - origin_.position.x) / (resolution_));
            int iy = static_cast<int>((std::get<1>(pos) - origin_.position.y) / (resolution_));
            
            if (ix >= 0 && ix < static_cast<int>(width_) && iy >= 0 && iy < static_cast<int>(height_)) 
            {
                int index = ix + iy * width_;
                
                    grid_msg.data[index] = 100;
                
            }
        }


        publisher_occupancy_grid->publish(grid_msg);
    }


   



    /*
    
        CALLBACKS.

    */


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        poseX_ = msg->pose.pose.position.x;
        poseY_ = msg->pose.pose.position.y;
        poseZ_ = msg->pose.pose.position.z;
    }

    void callback_cloud_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg)  
    {
       
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);
       
        for (const auto& point : pcl_cloud.points) 
        {
        
            float x = roundToMultiple(static_cast<float>(point.x), 0.05, decimals);
            float y = roundToMultiple(static_cast<float>(point.y), 0.05, decimals);

            std::pair<float, float> index = std::make_pair(x, y);

            if(verticesCloudMap.find(index) == verticesCloudMap.end())
            {
                verticesCloudMap[index] = false;
            }
    
            
            
        }
   
        createGraphFromPointCloud();
    }

    void check_parameters()
    {
        // Obtém os valores dos parâmetros
        auto new_distanceToObstacle = static_cast<float>(this->get_parameter("obstacle_graph_resolution").get_parameter_value().get<double>());
        auto new_maxSecurityDistance = static_cast<float>(this->get_parameter("maxSecurityDistance").get_parameter_value().get<double>());
        
        if (new_distanceToObstacle != distanceToObstacle_) 
        {
            verticesCloudMap.clear();
            verticesArbitrary.clear();
            distanceToObstacle_ = new_distanceToObstacle;
            RCLCPP_INFO(this->get_logger(), "obstacle_graph_resolution set to: %f", distanceToObstacle_);
        }

        if(new_maxSecurityDistance != maxSecurityDistance_)
        {
            verticesCloudMap.clear();
            verticesArbitrary.clear();
            maxSecurityDistance_ = new_maxSecurityDistance;
            RCLCPP_INFO(this->get_logger(), "maxSecurityDistance set to: %f", maxSecurityDistance_);
        }

     
  

    }


public:
    GraphPublisher()
    : Node("graph_publisher")
    {   
       
        this->declare_parameter<double>("obstacle_graph_resolution", 0.05);
        this->declare_parameter<double>("maxSecurityDistance", 0.15);


        distanceToObstacle_ = static_cast<float>(this->get_parameter("obstacle_graph_resolution").get_parameter_value().get<double>());
        maxSecurityDistance_ = static_cast<float>(this->get_parameter("maxSecurityDistance").get_parameter_value().get<double>());
       

        RCLCPP_INFO(this->get_logger(), "obstacle_graph_resolution is set to: %2f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "maxSecurityDistance is set to: %2f", maxSecurityDistance_);

       

      
        // Timer para verificar alterações nos parâmetros
        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GraphPublisher::check_parameters, this));

        decimals = countDecimals(distanceToObstacle_);


        publisher_occupancy_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid_map", 10);
        timer_occupancy_grid = this->create_wall_timer(100ms, std::bind(&GraphPublisher::occupancy_grid_callback, this));    
 

        publisher_vertices_arbitrary = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacles_vertices", 10);
        timer_vertices_arbitrary = this->create_wall_timer(100ms, std::bind(&GraphPublisher::publish_obstacles_vertices, this));
    
        subscription1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_map", 10, std::bind(&GraphPublisher::callback_cloud_map, this, std::placeholders::_1));

      
        subscription2_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GraphPublisher::odomCallback, this, std::placeholders::_1));

        resolution_ = distanceToObstacle_;  
        width_ = 1000;        
        height_ = 1000;
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
 
    rclcpp::spin(std::make_shared<GraphPublisher>());

    rclcpp::shutdown();

    return 0;
}