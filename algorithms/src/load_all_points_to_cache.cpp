#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include <fstream>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <filesystem>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// Hash specializations for std::pair
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

struct PairIntHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>{}(p.first) ^ (std::hash<int>{}(p.second) << 1);
    }
};

class GraphPublisher : public rclcpp::Node {
public:
    GraphPublisher() : Node("obstacle_vertices_publisher") {
        this->declare_parameter<std::string>("output_file", " ");

        filename = this->get_parameter("output_file").as_string();

        RCLCPP_INFO(this->get_logger(), "output_file is set to %s", filename.c_str());


        publisher_vertices_arbitrary = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacles_vertices", 10);
        timer_vertices_arbitrary = this->create_wall_timer(100ms, std::bind(&GraphPublisher::publish_obstacles_vertices, this));
    
    }

private:
    // ROS2 components
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr reload_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_vertices_arbitrary;
    rclcpp::TimerBase::SharedPtr timer_vertices_arbitrary;
    
    std::string filename = " ";

    static const size_t POINT_SIZE = sizeof(float) * 2; 
    static const size_t HEADER_SIZE = sizeof(int);
    static const size_t KEY_ENTRY_SIZE = sizeof(int) * 2 + sizeof(size_t); 
    
    // File and data management
    std::string frame_id_;
    std::unordered_set<std::pair<float, float>, PairHash> cachePoints;
    std::unordered_map<std::pair<int, int>, size_t, PairIntHash> index_map_;



    
    bool loadAllPointsToCache() {
       
        std::ifstream file(filename, std::ios::binary | std::ios::in);
        if (!file.is_open()) {
            return false;
        }

        cachePoints.clear();
        
        // Read total number of keys
        int total_keys;
        file.read(reinterpret_cast<char*>(&total_keys), sizeof(int));
        
        if (total_keys <= 0) {
            file.close();
            return true;
        }
        
        // Rebuild index map by reading index entries
        std::unordered_map<std::pair<int, int>, size_t, PairIntHash> temp_index_map;
        
        for (int i = 0; i < total_keys; ++i) {
            int x, y;
            size_t offset;
            file.read(reinterpret_cast<char*>(&x), sizeof(int));
            file.read(reinterpret_cast<char*>(&y), sizeof(int));
            file.read(reinterpret_cast<char*>(&offset), sizeof(size_t));
            
            std::pair<int, int> key = std::make_pair(x, y);
            temp_index_map[key] = offset;
        }
        
        // Read all points using the temporary index
        for (const auto& entry : temp_index_map) {
            size_t offset = entry.second;
            
            file.seekg(offset, std::ios::beg);
            
            int point_count;
            file.read(reinterpret_cast<char*>(&point_count), sizeof(int));
            
            if (point_count <= 0) continue;
            
            for (int i = 0; i < point_count; ++i) {
                float x, y;
                file.read(reinterpret_cast<char*>(&x), sizeof(float));
                file.read(reinterpret_cast<char*>(&y), sizeof(float));
                cachePoints.emplace(x, y);
            }
        }
        
        file.close();
        
       
        return true;
    }

    void reloadPoints() {
        loadAllPointsToCache();
    }

   

     void publish_obstacles_vertices()
    {
        loadAllPointsToCache();
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
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
  
        auto node = std::make_shared<GraphPublisher>();
        

        rclcpp::spin(node);
   
    
    rclcpp::shutdown();
    return 0;
}