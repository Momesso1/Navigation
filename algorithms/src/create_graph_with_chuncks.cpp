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
        size_t operator()(const std::pair<float, float>& p) const {
            size_t h1 = hash<float>()(p.first);
            size_t h2 = hash<float>()(p.second);
            return h1 ^ (h2 << 1);  
        }
    };

    template <>
    struct hash<std::pair<int, int>> {
        size_t operator()(const std::pair<int, int>& p) const {
            size_t h1 = hash<int>()(p.first);
            size_t h2 = hash<int>()(p.second);
            return h1 ^ (h2 << 1);  
        }
    };
}

struct PairHash {
    std::size_t operator()(const std::pair<float, float>& p) const {
        auto h1 = std::hash<float>{}(p.first);
        auto h2 = std::hash<float>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

class GraphPublisher : public rclcpp::Node {

private:
    struct CloudMapPoint
    {
        bool verified;
    };

    struct PairIntHash {
        std::size_t operator()(const std::pair<int, int>& p) const {
            return std::hash<int>{}(p.first) ^ (std::hash<int>{}(p.second) << 1);
        }
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

    std::string filename = " ";

    static const size_t POINT_SIZE = sizeof(float) * 2; 
    static const size_t HEADER_SIZE = sizeof(int);
    static const size_t KEY_ENTRY_SIZE = sizeof(int) * 2 + sizeof(size_t); 
    
    std::unordered_map<std::pair<int, int>, size_t, PairIntHash> indexMap; 
    
    float resolution_;
    unsigned int width_;
    unsigned int height_;
    float poseX_ = 0.0;
    float poseY_ = 0.0;
    float poseZ_ = 0.0;
    float maxSecurityDistance_, distanceToObstacle_, x_size = 1.0, y_size = 1.0;
    int decimals = 0;

    std::unordered_map<std::pair<float, float>, bool> verticesCloudMap;
    std::unordered_set<std::pair<float, float>> cachePoints;

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

    bool savePoints(const std::unordered_map<std::pair<int, int>, std::unordered_set<std::pair<float, float>, PairHash>, PairIntHash>& pointsByKey) 
    {
        std::unordered_map<std::pair<int, int>, std::unordered_set<std::pair<float, float>, PairHash>, PairIntHash> existingPoints;
        
        if (std::filesystem::exists(filename)) 
        {
            std::ifstream existingFile(filename, std::ios::binary | std::ios::in);
            if (existingFile.is_open()) 
            {
                int totalExistingKeys;
                existingFile.read(reinterpret_cast<char*>(&totalExistingKeys), sizeof(int));
                
                if (totalExistingKeys > 0) 
                {
                    std::unordered_map<std::pair<int, int>, size_t, PairIntHash> existingIndexMap;
                    
                    for (int i = 0; i < totalExistingKeys; ++i) 
                    {
                        int x, y;
                        size_t offset;
                        existingFile.read(reinterpret_cast<char*>(&x), sizeof(int));
                        existingFile.read(reinterpret_cast<char*>(&y), sizeof(int));
                        existingFile.read(reinterpret_cast<char*>(&offset), sizeof(size_t));
                        
                        std::pair<int, int> key = std::make_pair(x, y); // Chave 2D usando pair
                        existingIndexMap[key] = offset;
                    }
                    
                    for (const auto& entry : existingIndexMap) 
                    {
                        const std::pair<int, int>& key = entry.first;
                        size_t offset = entry.second;
                        
                        existingFile.seekg(offset, std::ios::beg);
                        
                        int pointCount;
                        existingFile.read(reinterpret_cast<char*>(&pointCount), sizeof(int));
                        
                        if (pointCount > 0)
                        {
                            std::unordered_set<std::pair<float, float>, PairHash> keyPoints; // 2D usando pair
                            
                            for (int i = 0; i < pointCount; ++i) {
                                float x, y;
                                existingFile.read(reinterpret_cast<char*>(&x), sizeof(float));
                                existingFile.read(reinterpret_cast<char*>(&y), sizeof(float));
                                keyPoints.emplace(x, y); // Ponto 2D usando pair
                            }
                            
                            existingPoints[key] = std::move(keyPoints);
                        }
                    }
                }

                existingFile.close();
            }
        }
        
        std::unordered_map<std::pair<int, int>, std::unordered_set<std::pair<float, float>, PairHash>, PairIntHash> combinedPoints = existingPoints;
        
        for (const auto& entry : pointsByKey) 
        {
            const std::pair<int, int>& key = entry.first;
            const std::unordered_set<std::pair<float, float>, PairHash>& newKeyPoints = entry.second;
            
            if (combinedPoints.find(key) != combinedPoints.end()) 
            {
                for (const auto& point : newKeyPoints) {
                    combinedPoints[key].insert(point);
                }
            } 
            else 
            {
                combinedPoints[key] = newKeyPoints;
            }
        }
        
        std::ofstream file(filename, std::ios::binary | std::ios::out);
        if (!file.is_open()) 
        {
            std::cerr << "Error opening file for writing: " << filename << std::endl;
            return false;
        }

        indexMap.clear();
        
        int totalKeys = static_cast<int>(combinedPoints.size());
        file.write(reinterpret_cast<const char*>(&totalKeys), sizeof(int));
        
        size_t dataStartOffset = sizeof(int) + (totalKeys * KEY_ENTRY_SIZE);
        size_t currentDataOffset = dataStartOffset;
        
        for (const auto& entry : combinedPoints) 
        {
            const std::pair<int, int>& key = entry.first;
            const std::unordered_set<std::pair<float, float>, PairHash>& keyPoints = entry.second;
            
            indexMap[key] = currentDataOffset;
            
            file.write(reinterpret_cast<const char*>(&key.first), sizeof(int));
            file.write(reinterpret_cast<const char*>(&key.second), sizeof(int));
            file.write(reinterpret_cast<const char*>(&currentDataOffset), sizeof(size_t));
            
            currentDataOffset += HEADER_SIZE + (keyPoints.size() * POINT_SIZE);
        }
        
        for (const auto& entry : combinedPoints) 
        {
            const std::unordered_set<std::pair<float, float>, PairHash>& keyPoints = entry.second;
            
            int pointCount = static_cast<int>(keyPoints.size());
            file.write(reinterpret_cast<const char*>(&pointCount), sizeof(int));
            
            for (const auto& point : keyPoints) 
            {
                file.write(reinterpret_cast<const char*>(&point.first), sizeof(float)); 
                file.write(reinterpret_cast<const char*>(&point.second), sizeof(float));
            }
        }

        file.close();
        
        int newPointsAdded = 0;
        for (const auto& entry : pointsByKey) 
        {
            newPointsAdded += entry.second.size();
        }
        
        return true;
    }


  

    void getAllPointsFromKey(std::pair<int, int> key) 
    {
        std::unordered_set<std::pair<float, float>, PairHash> keyPoints;
        
        auto indexIt = indexMap.find(key);
        if (indexIt == indexMap.end()) 
        {
            std::cerr << "Key (" << key.first << ", " << key.second << ") not found!" << std::endl;
            return;
        }

        std::ifstream file(filename, std::ios::binary | std::ios::in);
        if (!file.is_open()) 
        {
            std::cerr << "Error opening file for writing: " << filename << std::endl;
            return;
        }

        file.seekg(indexIt->second, std::ios::beg);
        
        int pointCount;
        file.read(reinterpret_cast<char*>(&pointCount), sizeof(int));
        
        if (pointCount <= 0) 
        {
            file.close();
            return;
        }

        for (int i = 0; i < pointCount; ++i) 
        {
            float x, y;
            file.read(reinterpret_cast<char*>(&x), sizeof(float));
            file.read(reinterpret_cast<char*>(&y), sizeof(float));
            std::pair<float, float> point = std::make_pair(x, y);
            cachePoints.insert(point);
        }
        
        file.close();
        return;
    }
    
    bool loadAllToCache() 
    {
        std::ifstream file(filename, std::ios::binary | std::ios::in);
        if (!file.is_open()) {
            std::cerr << "Error opening file for reading: " << filename << std::endl;
            return false;
        }

        cachePoints.clear();
        
        for (const auto& entry : indexMap) 
        {
            size_t offset = entry.second;
            
            file.seekg(offset, std::ios::beg);
            
            int pointCount;
            file.read(reinterpret_cast<char*>(&pointCount), sizeof(int));
            
            if (pointCount <= 0) continue;
            
            for (int i = 0; i < pointCount; ++i) 
            {
                float x, y;
                file.read(reinterpret_cast<char*>(&x), sizeof(float));
                file.read(reinterpret_cast<char*>(&y), sizeof(float));
                cachePoints.emplace(x, y);
            }
        }
        
        file.close();
        return true;
    }
  

    void createGraphFromPointCloud() 
    {
       
        std::unordered_map<std::pair<int, int>, std::unordered_set<std::pair<float, float>, PairHash>, PairIntHash> points;

          
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
                        
                        int x_index10 = static_cast<int>(std::floor(std::get<0>(index10) / x_size));
                        int y_index10 = static_cast<int>(std::floor(std::get<1>(index10) / y_size));

                        int x_index11 = static_cast<int>(std::floor(std::get<0>(index11) / x_size));
                        int y_index11 = static_cast<int>(std::floor(std::get<1>(index11) / y_size));

                        int x_index12 = static_cast<int>(std::floor(std::get<0>(index12) / x_size));
                        int y_index12 = static_cast<int>(std::floor(std::get<1>(index12) / y_size));

                        int x_index13 = static_cast<int>(std::floor(std::get<0>(index13) / x_size));
                        int y_index13 = static_cast<int>(std::floor(std::get<1>(index13) / y_size));

                        std::pair<int, int> xy_index10 = std::make_pair(x_index10, y_index10);
                        std::pair<int, int> xy_index11 = std::make_pair(x_index11, y_index11);
                        std::pair<int, int> xy_index12 = std::make_pair(x_index12, y_index12);
                        std::pair<int, int> xy_index13 = std::make_pair(x_index13, y_index13);

                    
                        points[xy_index10].insert(index10);    
                        points[xy_index11].insert(index11);                    
                        points[xy_index12].insert(index12);                    
                        points[xy_index13].insert(index13);                    
            
                        
                    }

                    opa++;

                    toma += distanceToObstacle_;
                }
              
            }
            
        }

        savePoints(points);
   
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
        cloud_msg1.width = cachePoints.size(); 
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
        for (const auto& vertex : cachePoints) {
           
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
        if (cachePoints.empty()) return;

        // encontrar limites do mapa
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();

        for (const auto &pos : cachePoints)
        {
            min_x = std::min(min_x, pos.first);
            min_y = std::min(min_y, pos.second);
            max_x = std::max(max_x, pos.first);
            max_y = std::max(max_y, pos.second);
        }

        // arredondar para múltiplos da resolução
        min_x = std::floor(min_x / resolution_) * resolution_;
        min_y = std::floor(min_y / resolution_) * resolution_;
        max_x = std::ceil(max_x / resolution_) * resolution_;
        max_y = std::ceil(max_y / resolution_) * resolution_;

        unsigned int width = static_cast<unsigned int>((max_x - min_x) / resolution_) + 1;
        unsigned int height = static_cast<unsigned int>((max_y - min_y) / resolution_) + 1;

        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = this->get_clock()->now();
        grid_msg.header.frame_id = "map";

        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = width;
        grid_msg.info.height = height;

        geometry_msgs::msg::Pose origin;
        origin.position.x = min_x;
        origin.position.y = min_y;
        origin.position.z = 0.0;
        origin.orientation.w = 1.0;
        grid_msg.info.origin = origin;

        grid_msg.data.assign(width * height, 0);

        for (const auto &pos : cachePoints)
        {
            int ix = static_cast<int>((pos.first - min_x) / resolution_);
            int iy = static_cast<int>((pos.second - min_y) / resolution_);

            if (ix >= 0 && ix < static_cast<int>(width) &&
                iy >= 0 && iy < static_cast<int>(height))
            {
                int index = ix + iy * width;
                grid_msg.data[index] = 100; // ocupado
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
        auto new_distanceToObstacle = static_cast<float>(this->get_parameter("create_graph_distance_to_obstacle").get_parameter_value().get<double>());
        auto new_maxSecurityDistance = static_cast<float>(this->get_parameter("maxSecurityDistance").get_parameter_value().get<double>());
        
        if (new_distanceToObstacle != distanceToObstacle_) 
        {
            verticesCloudMap.clear();
            distanceToObstacle_ = new_distanceToObstacle;
            RCLCPP_INFO(this->get_logger(), "create_graph_distance_to_obstacle set to: %f", distanceToObstacle_);
        }

        if(new_maxSecurityDistance != maxSecurityDistance_)
        {
            verticesCloudMap.clear();
            maxSecurityDistance_ = new_maxSecurityDistance;
            RCLCPP_INFO(this->get_logger(), "maxSecurityDistance set to: %f", maxSecurityDistance_);
        }

     
  

    }


public:
    GraphPublisher()
    : Node("graph_publisher")
    {   
        this->declare_parameter<std::string>("output_file", "/home/momesso/autonomous/src/navigation_2d_with_file/config/pontos_convertidos.bin");
        this->declare_parameter<double>("create_graph_distance_to_obstacle", 0.05);
        this->declare_parameter<double>("maxSecurityDistance", 0.15);
        this->declare_parameter<double>("x_size", 1.0);
        this->declare_parameter<double>("y_size", 1.0);


        filename = this->get_parameter("output_file").as_string();
        distanceToObstacle_ = static_cast<float>(this->get_parameter("create_graph_distance_to_obstacle").get_parameter_value().get<double>());
        maxSecurityDistance_ = static_cast<float>(this->get_parameter("maxSecurityDistance").get_parameter_value().get<double>());
        x_size = static_cast<float>(this->get_parameter("x_size").get_parameter_value().get<double>());
        y_size = static_cast<float>(this->get_parameter("y_size").get_parameter_value().get<double>());

        RCLCPP_INFO(this->get_logger(), "output_file is set to %s", filename.c_str());
        RCLCPP_INFO(this->get_logger(), "create_graph_distance_to_obstacle is set to %2f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "maxSecurityDistance is set to %2f", maxSecurityDistance_);
        RCLCPP_INFO(this->get_logger(), "x_size is set to %2f", x_size);
        RCLCPP_INFO(this->get_logger(), "y_size is set to %2f", y_size);

      
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