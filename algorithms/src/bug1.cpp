#include <string>
#include <random>
#include <algorithm>
#include <geometry_msgs/msg/point.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <map>
#include <stack>
#include <unordered_map>
#include <optional>
#include <iostream>
#include <climits>
#include <iomanip>
#include <thread>
#include <queue>
#include <tuple>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>                       
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cmath>
#include <cstring>
#include <utility> 
#include <iomanip>
#include <filesystem>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"


using namespace std::chrono_literals;

namespace std 
{
    template <>
    struct hash<std::tuple<float, float, float>> 
    {
        size_t operator()(const std::tuple<float, float, float>& t) const 
        {
            size_t h1 = hash<float>()(std::get<0>(t));
            size_t h2 = hash<float>()(std::get<1>(t));
            size_t h3 = hash<float>()(std::get<2>(t));
            
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
}

namespace std {
    template<>
    struct hash<std::tuple<std::pair<int, int>, bool>> {
        size_t operator()(const std::tuple<std::pair<int, int>, bool>& t) const {
            const auto& p = std::get<0>(t);
            bool b = std::get<1>(t);
            size_t h1 = std::hash<int>{}(p.first);
            size_t h2 = std::hash<int>{}(p.second);
            size_t h3 = std::hash<bool>{}(b);
            size_t seed = h1;
            seed ^= h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= h3 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
}

template <typename T1, typename T2>
struct pair_hash {
    std::size_t operator ()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1); 
    }
};



template<typename T1, typename T2, typename T3>
std::ostream& operator<<(std::ostream& os, const std::tuple<T1, T2, T3>& t) {
    os << "(" << std::get<0>(t) << ", " 
       << std::get<1>(t) << ", " 
       << std::get<2>(t) << ")";
    return os;
}



class AStar : public rclcpp::Node {

private:


    struct Vertex {
        int key;
        float x, y, z;
    };

    struct VertexDijkstra {
        float x, y;
        float orientation_x, orientation_y, orientation_z;
        float orientation_w;
    };

    struct Destinos {
        float x, y, z;
        float orientation_x, orientation_y, orientation_z;
        float orientation_w;
    };

    struct Edge {
        int v1, v2;
    };

    struct CompareWithTieBreaker {
        bool operator()(const std::pair<float, int>& a, const std::pair<float, int>& b) const {
            if (std::abs(a.first - b.first) < 1e-6) {
                return a.second > b.second;
            }
            return a.first > b.first;
        }
    };
    struct PairHash {
        std::size_t operator()(const std::pair<float, float>& p) const {
            auto h1 = std::hash<float>{}(p.first);
            auto h2 = std::hash<float>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };

    struct TupleCompare {
        bool operator()(const std::pair<float, std::tuple<float, float, float>>& a, 
                        const std::pair<float, std::tuple<float, float, float>>& b) const {
            return a.first > b.first;
        }
    };

    struct PositionProb {
        float x;
        float y;
        float prob;
    };
 

    //Publishers.
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_nav_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_path_without_filter_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_created_vertices;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_created_vertices1;

    //Subscriptions.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_navigable_removed_vertices;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription3_;

    //Timers.
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_path_;
    rclcpp::TimerBase::SharedPtr timer_visualize_path_;
    rclcpp::TimerBase::SharedPtr parameterTimer;    
    

    size_t i_ = 0; 
    int diagonalEdges_;
    float pose_x_ = 0.0, pose_y_ = -0.2, pose_z_ = 0.0;
    float distanceToObstacle_;
    int decimals = 0, iterations_before_verification = 10;

    std::vector<VertexDijkstra> verticesDestino_;
    std::vector<VertexDijkstra> verticesDijkstra;
    std::vector<VertexDijkstra> path_points;

    std::unordered_set<std::pair<float, float>, PairHash> obstaclesVertices;

    inline float round_to_multiple(float value, float multiple, int decimals) 
    {
        if (multiple == 0.0) return value; 
        
        float result = std::round(value / multiple) * multiple;
        float factor = std::pow(10.0, decimals);
        result = std::round(result * factor) / factor;
        
        return result;
    }
    

    int count_decimals(float number) 
    {
      
        float fractional = std::fabs(number - std::floor(number));
        int decimals = 0;
        const float epsilon = 1e-9; 
    
  
        while (fractional > epsilon && decimals < 20) {
            fractional *= 10;
            fractional -= std::floor(fractional);
            decimals++;
        }
        return decimals;
    }
    

    

    std::vector<std::array<float, 3>> get_offsets(float distanceToObstacle) {
        return {
           
            {-distanceToObstacle, 0.0, 0.0},
            {0.0, distanceToObstacle, 0.0},
            {distanceToObstacle, 0.0, 0.0},
            {0.0, -distanceToObstacle, 0.0},
            
            {-distanceToObstacle, distanceToObstacle, 0.0}, 
            {distanceToObstacle, distanceToObstacle, 0.0},
            {distanceToObstacle, -distanceToObstacle, 0.0},
          {-distanceToObstacle, -distanceToObstacle, 0.0},

        };
    }
    std::unordered_set<std::pair<float, float>, PairHash> visited_obstacles;
    std::unordered_set<std::pair<float, float>, PairHash> visited_points;

    std::vector<std::pair<float, float>> bug1(std::pair<float, float> initial_pose, std::pair<float, float> goal_pose)
    {
        path_points.clear();
        verticesDijkstra.clear();

        publisher_path_without_filter();
        publisher_dijkstra();
        publisher_dijkstra_path();

        std::pair<float, float> actual_obstacle = straight_line_to_obstacle(initial_pose, goal_pose);
        std::vector<std::pair<float, float>> path;

        path.push_back(initial_pose);

        std::pair<float, float> actual_point;
        std::pair<float, float> closer_point;

        auto offsets1 = get_offsets(distanceToObstacle_);

        float new_x = 0.0, new_y = 0.0;
        
        auto heuristic = [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
            float x1 = std::get<0>(a);
            float y1 = std::get<1>(a);
            
            float x2 = std::get<0>(b);
            float y2 = std::get<1>(b);
            
            return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
        };

        std::unordered_map<std::pair<float, float>, std::unordered_set<std::pair<float, float>, PairHash>, PairHash> adjacency_list;
        std::pair<float, float> first_obstacle;

         
      
        do
        {
            std::pair<float, float> temp;
            int i = 0;
            
            // visited_obstacles.clear();

            // visited_points.clear();
            adjacency_list.clear();
           

            visited_obstacles.insert(actual_obstacle);
            first_obstacle = actual_obstacle;

            float distance = std::numeric_limits<float>::infinity();
            
            for (int a = 0; a < 8; a++) 
            {
                new_x = round_to_multiple(std::get<0>(actual_obstacle) + offsets1[a][0], distanceToObstacle_, decimals);
                new_y = round_to_multiple(std::get<1>(actual_obstacle) + offsets1[a][1], distanceToObstacle_, decimals);

                std::pair<float, float> neighbor_tuple = std::make_pair(static_cast<float>(new_x), static_cast<float>(new_y)); 
                
                adjacency_list[actual_obstacle].insert(neighbor_tuple);
            }
            
            for (const auto& current_point : adjacency_list[actual_obstacle])
            {
                for (int a = 0; a < 8; a++)  
                {
                    new_x = round_to_multiple(std::get<0>(current_point) + offsets1[a][0], distanceToObstacle_, decimals);
                    new_y = round_to_multiple(std::get<1>(current_point) + offsets1[a][1], distanceToObstacle_, decimals);
            
                    std::pair<float, float> neighbor_tuple = std::make_pair(static_cast<float>(new_x), static_cast<float>(new_y)); 
                    
                    if (adjacency_list[actual_obstacle].find(neighbor_tuple) != adjacency_list[actual_obstacle].end() && obstaclesVertices.find(neighbor_tuple) == obstaclesVertices.end() && visited_points.find(neighbor_tuple) == visited_points.end() && obstaclesVertices.find(current_point) != obstaclesVertices.end() && visited_obstacles.find(current_point) == visited_obstacles.end())
                    {
                        actual_obstacle = current_point;
                        actual_point = neighbor_tuple;
                        visited_obstacles.insert(current_point);
                        visited_points.insert(neighbor_tuple);
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                        float temp_distance = heuristic(neighbor_tuple, goal_pose);

                        if(temp_distance < distance)
                        {
                            distance = temp_distance;
                            closer_point = neighbor_tuple;
                        }
                        
                        publish_created_vertices();
                        publish_created_vertices2();
                        break;
                    }
                }
            }


            while(i < 5000)
            {

                for (int a = 0; a < 8; a++) 
                {
                    new_x = round_to_multiple(std::get<0>(actual_obstacle) + offsets1[a][0], distanceToObstacle_, decimals);
                    new_y = round_to_multiple(std::get<1>(actual_obstacle) + offsets1[a][1], distanceToObstacle_, decimals);

                    std::pair<float, float> neighbor_tuple = std::make_pair(static_cast<float>(new_x), static_cast<float>(new_y)); 
                    
                    adjacency_list[actual_obstacle].insert(neighbor_tuple);
                }

                bool found = false;

                for (const auto& current_point : adjacency_list[actual_obstacle])
                {
                    for (int a = 0; a < 8; a++)  
                    {
                        new_x = round_to_multiple(std::get<0>(current_point) + offsets1[a][0], distanceToObstacle_, decimals);
                        new_y = round_to_multiple(std::get<1>(current_point) + offsets1[a][1], distanceToObstacle_, decimals);
                
                        std::pair<float, float> neighbor_tuple = std::make_pair(static_cast<float>(new_x), static_cast<float>(new_y)); 
                        
                        
                        if (adjacency_list[actual_obstacle].find(neighbor_tuple) != adjacency_list[actual_obstacle].end() && obstaclesVertices.find(neighbor_tuple) == obstaclesVertices.end() && visited_points.find(neighbor_tuple) == visited_points.end() && obstaclesVertices.find(current_point) != obstaclesVertices.end() && visited_obstacles.find(current_point) == visited_obstacles.end())
                        {
                            actual_obstacle = current_point;
                            actual_point = neighbor_tuple;
                            visited_obstacles.insert(current_point);
                            visited_points.insert(neighbor_tuple);
                            std::this_thread::sleep_for(std::chrono::milliseconds(2));
                            

                            publish_created_vertices();
                            publish_created_vertices2();
                            found = true;
                            break;
                        }

                    }

                }

                if(found == false)
                {
                    
                    for (const auto& current_point : adjacency_list[actual_obstacle])
                    {
                        for (int a = 0; a < 8; a++)  
                        {
                            new_x = round_to_multiple(std::get<0>(current_point) + offsets1[a][0], distanceToObstacle_, decimals);
                            new_y = round_to_multiple(std::get<1>(current_point) + offsets1[a][1], distanceToObstacle_, decimals);
                    
                            std::pair<float, float> neighbor_tuple = std::make_pair(static_cast<float>(new_x), static_cast<float>(new_y)); 
                            
                            
                            if (adjacency_list[actual_obstacle].find(neighbor_tuple) != adjacency_list[actual_obstacle].end() 
                            && obstaclesVertices.find(neighbor_tuple) == obstaclesVertices.end() && visited_points.find(neighbor_tuple) != visited_points.end() 
                            && obstaclesVertices.find(current_point) != obstaclesVertices.end() && visited_obstacles.find(current_point) == visited_obstacles.end())
                            {
                                actual_obstacle = current_point;
                                actual_point = neighbor_tuple;
                                visited_obstacles.insert(current_point);
                                visited_points.insert(neighbor_tuple);
                                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                                

                                publish_created_vertices();
                                publish_created_vertices2();
                                break;
                            }

                        }
                    } 
                }
            

                i++;

            }

            for (const auto& current_point : visited_obstacles)
            {
                for (int a = 0; a < 8; a++)  
                {
                    new_x = round_to_multiple(std::get<0>(current_point) + offsets1[a][0], distanceToObstacle_, decimals);
                    new_y = round_to_multiple(std::get<1>(current_point) + offsets1[a][1], distanceToObstacle_, decimals);
            
                    std::pair<float, float> neighbor_tuple = std::make_pair(static_cast<float>(new_x), static_cast<float>(new_y)); 
                    
                    
                    if (obstaclesVertices.find(neighbor_tuple) == obstaclesVertices.end() && visited_points.find(neighbor_tuple) == visited_points.end())
                    {
                        actual_point = neighbor_tuple;
                        visited_points.insert(neighbor_tuple);
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                        
                        
                        publish_created_vertices2();
                    
                    }
                    else if(obstaclesVertices.find(neighbor_tuple) == obstaclesVertices.end())
                    {
                        float temp_distance = heuristic(neighbor_tuple, goal_pose);

                        if(temp_distance < distance)
                        {
                            distance = temp_distance;
                            closer_point = neighbor_tuple;
                        }
                    }

                }
            }

            std::unordered_map<std::pair<float, float>, std::vector<std::pair<float, float>>, PairHash> adjacency_visited_points_list;

            for (const auto& current_point : visited_points)
            {
                for (int a = 0; a < 8; a++)  
                {
                    new_x = round_to_multiple(std::get<0>(current_point) + offsets1[a][0], distanceToObstacle_, decimals);
                    new_y = round_to_multiple(std::get<1>(current_point) + offsets1[a][1], distanceToObstacle_, decimals);
            
                    std::pair<float, float> neighbor_tuple = std::make_pair(static_cast<float>(new_x), static_cast<float>(new_y)); 
                    
                    
                    if (visited_points.find(neighbor_tuple) != visited_points.end())
                    {
                        adjacency_visited_points_list[current_point].push_back(neighbor_tuple);
                    }

                }
            }



            std::unordered_set<std::pair<float, float>, PairHash> visited;
            
            std::pair<float, float> current = first_obstacle;


            int f = 0;

            for (const auto& current_point : adjacency_list[current])
            {
                if(visited_points.find(current_point) != visited_points.end() && visited.find(current_point) == visited.end() && obstaclesVertices.find(current_point) == obstaclesVertices.end())
                {
                    current = current_point;
                    path.push_back(current);
                    visited.insert(current_point);
                    
                    break;
                }
            }

            do
            {
                for (const auto& current_point : adjacency_visited_points_list[current])
                {
                    if(visited_points.find(current_point) != visited_points.end() && visited.find(current_point) == visited.end() && obstaclesVertices.find(current_point) == obstaclesVertices.end())
                    {
                        current = current_point;
                        path.push_back(current);
                        visited.insert(current_point);
                        
                        break;
                    }
                }

                f++;

            } while (current != closer_point);

            
            actual_obstacle = straight_line_to_obstacle(closer_point, goal_pose);

        } while (actual_obstacle != goal_pose);
        
        path.push_back(goal_pose);



        publish_created_vertices();
        publish_created_vertices2();
        
        
        
        visited_obstacles.clear();
        visited_points.clear();
        return path;
    } 

    std::pair<float, float> straight_line_to_obstacle(std::pair<float, float> start_tuple, std::pair<float, float> goal_tuple)
    {
        
        
        std::pair<float, float> A {
            std::get<0>(start_tuple),
            std::get<1>(start_tuple),
        };
        std::pair<float, float> B {
            std::get<0>(goal_tuple),
            std::get<1>(goal_tuple),
        };

        float ax = std::get<0>(A), ay = std::get<1>(A);
        float bx = std::get<0>(B), by = std::get<1>(B);

        float dx = bx - ax, dy = by - ay;
        float distance = std::sqrt(dx * dx + dy * dy);

        float ux = dx / distance;
        float uy = dy / distance;

        float step = distanceToObstacle_;
        float t = 0.0f;
        bool obstacleFound = false;
        auto offsets1 = get_offsets(distanceToObstacle_);
        
        std::vector<std::pair<float, float>> path;

        std::pair<float, float> before_obstacle;

        while (t < distance && obstacleFound == false) 
        {
            std::tuple<float, float, float> point;
            std::get<0>(point) = ax + t * ux;
            std::get<1>(point) = ay + t * uy;

            float new_x = round_to_multiple(std::get<0>(point), distanceToObstacle_, decimals);
            float new_y = round_to_multiple(std::get<1>(point), distanceToObstacle_, decimals);

            std::pair<float, float> actual_pose = std::make_pair(static_cast<float>(new_x), static_cast<float>(new_y));

            std::pair<float, float> neighbor_tuple = actual_pose;
            
            

            if (obstaclesVertices.find(neighbor_tuple) != obstaclesVertices.end()) 
            {
                
                obstacleFound = true;
                return neighbor_tuple;
            }

         

            t += step;
        }

        
        
        return goal_tuple;
    }


    std::vector<std::pair<float, float>> straight_line(std::pair<float, float> start_tuple, std::pair<float, float> goal_tuple)
    {
        
        
        std::pair<float, float> A {
            std::get<0>(start_tuple),
            std::get<1>(start_tuple),
        };
        std::pair<float, float> B {
            std::get<0>(goal_tuple),
            std::get<1>(goal_tuple),
        };

        float ax = std::get<0>(A), ay = std::get<1>(A);
        float bx = std::get<0>(B), by = std::get<1>(B);

        float dx = bx - ax, dy = by - ay;
        float distance = std::sqrt(dx * dx + dy * dy);

        float ux = dx / distance;
        float uy = dy / distance;

        float step = distanceToObstacle_;
        float t = 0.0f;
        bool obstacleFound = false;
        auto offsets1 = get_offsets(distanceToObstacle_);
        
        std::vector<std::pair<float, float>> path;

        while (t < distance && obstacleFound == false) 
        {
            std::tuple<float, float, float> point;
            std::get<0>(point) = ax + t * ux;
            std::get<1>(point) = ay + t * uy;

            float new_x = round_to_multiple(std::get<0>(point), distanceToObstacle_, decimals);
            float new_y = round_to_multiple(std::get<1>(point), distanceToObstacle_, decimals);

            std::pair<float, float> neighbor_tuple = std::make_pair(static_cast<float>(new_x), static_cast<float>(new_y));
            
            path.push_back(neighbor_tuple);
            if (obstaclesVertices.find(neighbor_tuple) != obstaclesVertices.end()) 
            {
                
                obstacleFound = true;
                break;
            }

            t += step;
        }

        
        if(obstacleFound == true)
        {
            return {};
        }
        else
        {
            return path;
        }
    }

    

    void store_edges_in_path(std::vector<std::pair<float, float>>& path) 
    {
        verticesDijkstra.clear();
        path_points.clear();

        for (size_t i = 0; i < path.size(); i++) 
        {
            VertexDijkstra vertex;
            
            vertex.x = std::get<0>(path[i]);
            vertex.y = std::get<1>(path[i]);

            if (i < path.size() - 1) 
            {
                const std::pair<float, float>& current_vertex = path[i];
                const std::pair<float, float>& next_vertex = path[i + 1];

                float dx = std::get<0>(next_vertex) - std::get<0>(current_vertex);
                float dy = std::get<1>(next_vertex) - std::get<1>(current_vertex);
                float distance = std::sqrt(dx * dx + dy * dy);

                if (distance > 0.0f) {
                    dx /= distance;
                    dy /= distance;
                }

                Eigen::Vector3f direction(dx, dy, 0.0f);
                Eigen::Vector3f reference(1.0f, 0.0f, 0.0f); 

                Eigen::Quaternionf quaternion = Eigen::Quaternionf::FromTwoVectors(reference, direction);

                vertex.orientation_x = quaternion.x();
                vertex.orientation_y = quaternion.y();
                vertex.orientation_z = quaternion.z();
                vertex.orientation_w = quaternion.w();
            } 
            else 
            {
                vertex.orientation_x = 0.0;
                vertex.orientation_y = 0.0;
                vertex.orientation_z = 0.0;
                vertex.orientation_w = 1.0;
            }

            path_points.push_back(vertex);
        }

        publisher_path_without_filter();
        
        if (path.empty()) {
            return;
        }

        auto start_time1_ = std::chrono::high_resolution_clock::now();
        int k = 0;
        int iterations = 0, iterations2 = 0;

        while (k < static_cast<int>(path.size()) - 1) 
        {
            bool shortcutFound = false;
            for (int i = static_cast<int>(path.size()) - 1; i > k; i--) 
            {
                std::pair<float, float> A {
                    std::get<0>(path[k]),
                    std::get<1>(path[k]),
                };
                std::pair<float, float> B {
                    std::get<0>(path[i]),
                    std::get<1>(path[i]),
                };
        
                float ax = std::get<0>(A), ay = std::get<1>(A);
                float bx = std::get<0>(B), by = std::get<1>(B);
        
                float dx = bx - ax, dy = by - ay;
                float distance = std::sqrt(dx * dx + dy * dy);
        
                if (distance == 0) 
                {
                    continue;
                }
        
                float ux = dx / distance;
                float uy = dy / distance;
        
                float step = distanceToObstacle_;
                float t = 0.0f;
                bool obstacleFound = false;
                auto offsets1 = get_offsets(distanceToObstacle_);
        
                while (t < distance && obstacleFound == false) 
                {
                    std::pair<float, float> point;
                    std::get<0>(point) = ax + t * ux;
                    std::get<1>(point) = ay + t * uy;
        
                    double new_x = round_to_multiple(std::get<0>(point), distanceToObstacle_, decimals);
                    double new_y = round_to_multiple(std::get<1>(point), distanceToObstacle_, decimals);
        
                    std::pair<float, float> neighbor_tuple = std::make_pair(static_cast<float>(new_x), static_cast<float>(new_y));
                    
                    iterations++;
                    if (obstaclesVertices.find(neighbor_tuple) != obstaclesVertices.end()) 
                    {
                        obstacleFound = true;
                        break;
                    }
                

                    t += step;
                }
        
                if (obstacleFound == false) 
                {
                    path.erase(path.begin() + k + 1, path.begin() + i);
                    shortcutFound = true;
                    break;  
                }
            }
        
            iterations2++;

            if (shortcutFound == true)
            {
                k++;
            } 
            else if(shortcutFound == false)
            {
                break;
            }
        }

      
        auto end_time1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration1 = end_time1 - start_time1_; 

        RCLCPP_INFO(this->get_logger(), "Bug 1 filter execution time: %.10f", duration1.count());

       

        for (size_t i = 0; i < path.size(); i++) 
        {
            VertexDijkstra vertex;
            
            vertex.x = std::get<0>(path[i]);
            vertex.y = std::get<1>(path[i]);

            if (i < path.size() - 1) 
            {
                const std::pair<float, float>& current_vertex = path[i];
                const std::pair<float, float>& next_vertex = path[i + 1];

                float dx = std::get<0>(next_vertex) - std::get<0>(current_vertex);
                float dy = std::get<1>(next_vertex) - std::get<1>(current_vertex);
                float distance = std::sqrt(dx * dx + dy * dy);

                if (distance > 0.0f) {
                    dx /= distance;
                    dy /= distance;
                }

                Eigen::Vector3f direction(dx, dy, 0.0f);
                Eigen::Vector3f reference(1.0f, 0.0f, 0.0f); 

                Eigen::Quaternionf quaternion = Eigen::Quaternionf::FromTwoVectors(reference, direction);

                vertex.orientation_x = quaternion.x();
                vertex.orientation_y = quaternion.y();
                vertex.orientation_z = quaternion.z();
                vertex.orientation_w = quaternion.w();
            } 
            else 
            {
                vertex.orientation_x = 0.0;
                vertex.orientation_y = 0.0;
                vertex.orientation_z = 0.0;
                vertex.orientation_w = 1.0;
            }

            verticesDijkstra.push_back(vertex);
        }


        publisher_dijkstra();
        publisher_dijkstra_path();
    }


    /*

        PUBLISHERS.

    */


    
    void publisher_dijkstra()
    {   
        geometry_msgs::msg::PoseArray message;
        message.header.stamp = this->now();
        message.header.frame_id = "map";

        for (const auto& vertex : verticesDijkstra) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = vertex.x;
            pose.position.y = vertex.y;
            pose.position.z = 0.0;
            pose.orientation.x = vertex.orientation_x;
            pose.orientation.y = vertex.orientation_y;
            pose.orientation.z = vertex.orientation_z;
            pose.orientation.w = vertex.orientation_w; 
            message.poses.push_back(pose);
        }

        publisher_path_->publish(message);
    }

    void publisher_path_without_filter()
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        for (const auto& vertex : path_points)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = "map";
            
            pose_stamped.pose.position.x = vertex.x;
            pose_stamped.pose.position.y = vertex.y;
            pose_stamped.pose.position.z = 0.0  ;
            pose_stamped.pose.orientation.x = vertex.orientation_x;
            pose_stamped.pose.orientation.y = vertex.orientation_y;
            pose_stamped.pose.orientation.z = vertex.orientation_z;
            pose_stamped.pose.orientation.w = vertex.orientation_w;
            
            path_msg.poses.push_back(pose_stamped);
        }

        publisher_path_without_filter_->publish(path_msg);
    }

    void publisher_dijkstra_path()
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        for (const auto& vertex : verticesDijkstra)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = "map";
            
            pose_stamped.pose.position.x = vertex.x;
            pose_stamped.pose.position.y = vertex.y;
            pose_stamped.pose.position.z = 0.0  ;
            pose_stamped.pose.orientation.x = vertex.orientation_x;
            pose_stamped.pose.orientation.y = vertex.orientation_y;
            pose_stamped.pose.orientation.z = vertex.orientation_z;
            pose_stamped.pose.orientation.w = vertex.orientation_w;
            
            path_msg.poses.push_back(pose_stamped);
        }

        publisher_nav_path_->publish(path_msg);
    }

    void publish_created_vertices()
    {
        sensor_msgs::msg::PointCloud2 cloud_msgs_created_vertices;
        cloud_msgs_created_vertices.header.stamp = this->get_clock()->now();
        cloud_msgs_created_vertices.header.frame_id = "map";

        cloud_msgs_created_vertices.height = 1; 
        cloud_msgs_created_vertices.width = visited_obstacles.size(); 
        cloud_msgs_created_vertices.is_dense = true;
        cloud_msgs_created_vertices.is_bigendian = false;
        cloud_msgs_created_vertices.point_step = 3 * sizeof(float); 
        cloud_msgs_created_vertices.row_step = cloud_msgs_created_vertices.point_step * cloud_msgs_created_vertices.width;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msgs_created_vertices);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cloud_msgs_created_vertices.width);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msgs_created_vertices, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msgs_created_vertices, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msgs_created_vertices, "z");
        for (const auto& vertex : visited_obstacles) 
        {
           
                *iter_x = std::get<0>(vertex);
                *iter_y = std::get<1>(vertex);
                *iter_z = 0.0;

                ++iter_x;
                ++iter_y;
                ++iter_z;    
        }


         publisher_created_vertices->publish(cloud_msgs_created_vertices);
    }

    void publish_created_vertices2()
    {
        sensor_msgs::msg::PointCloud2 cloud_msgs_created_vertices;
        cloud_msgs_created_vertices.header.stamp = this->get_clock()->now();
        cloud_msgs_created_vertices.header.frame_id = "map";

        cloud_msgs_created_vertices.height = 1; 
        cloud_msgs_created_vertices.width = visited_points.size(); 
        cloud_msgs_created_vertices.is_dense = true;
        cloud_msgs_created_vertices.is_bigendian = false;
        cloud_msgs_created_vertices.point_step = 3 * sizeof(float); 
        cloud_msgs_created_vertices.row_step = cloud_msgs_created_vertices.point_step * cloud_msgs_created_vertices.width;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msgs_created_vertices);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cloud_msgs_created_vertices.width);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msgs_created_vertices, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msgs_created_vertices, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msgs_created_vertices, "z");
        for (const auto& vertex : visited_points) 
        {
           
                *iter_x = std::get<0>(vertex);
                *iter_y = std::get<1>(vertex);
                *iter_z = 0.0;

                ++iter_x;
                ++iter_y;
                ++iter_z;    
        }


         publisher_created_vertices1->publish(cloud_msgs_created_vertices);
    }


    /*
    
        CALLBACKS.

    */
   

    void callback_destinations(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
    {
        verticesDestino_.clear();
        for (const auto& pose_in : msg->poses) {
            VertexDijkstra destino;

            destino.x = pose_in.position.x;
            destino.y = pose_in.position.y;

            destino.orientation_x = pose_in.orientation.x;
            destino.orientation_y = pose_in.orientation.y;
            destino.orientation_z = pose_in.orientation.z;
            destino.orientation_w = pose_in.orientation.w;

            verticesDestino_.push_back(destino);
        }
        
         
        if(!verticesDestino_.empty())
        {
        
            float dx = pose_x_ - static_cast<float>(verticesDestino_[i_].x);
            float dy = pose_y_ - static_cast<float>(verticesDestino_[i_].y);

            std::pair<float, float> initial_pose = {static_cast<float>(pose_x_), static_cast<float>(pose_y_)};
            std::pair<float, float> goal_pose = {static_cast<float>(verticesDestino_[i_].x), static_cast<float>(verticesDestino_[i_].y)};

            float distance_to_goal = sqrt(dx * dx + dy * dy);

            auto start_time_ = std::chrono::high_resolution_clock::now();

          
            if(!obstaclesVertices.empty())
            {
                 std::vector<std::pair<float, float>> shortestPath = bug1(initial_pose, goal_pose);

                  visited_points.clear();
                visited_obstacles.clear();

                publish_created_vertices();
                publish_created_vertices2();
                    
            
                store_edges_in_path(shortestPath);

                std::this_thread::sleep_for(std::chrono::seconds(2));
            
                auto end_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<float> duration = end_time - start_time_;  
                

                if(distance_to_goal <= distanceToObstacle_)
                {
                    i_ = i_ + 1;
                }

                if(i_ == verticesDestino_.size())
                {
                    i_ = 0;
                }
                
                // RCLCPP_INFO(this->get_logger(),"Bug1 execution time: %.10f", duration.count());
            }
        }
    }

    void callback_removed_navigable_vertices(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
   
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            float x = *iter_x;
            float y = *iter_y;

            std::pair<float, float> index = std::make_pair(x, y);
    
            obstaclesVertices.insert(index);
            
          
        }
        
        
    }

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        pose_x_ = msg->pose.pose.position.x;
        pose_y_ = msg->pose.pose.position.y;
        pose_z_ = 0.0;
    }


   
    void check_parameters()
    {
      
        auto new_distanceToObstacle = static_cast<float>(this->get_parameter("path_resolution").get_parameter_value().get<double>());
        auto new_diagonalEdges = this->get_parameter("diagonalEdges").get_parameter_value().get<int>();
        
        if (new_distanceToObstacle != distanceToObstacle_) 
        {
            distanceToObstacle_ = new_distanceToObstacle;
            std::cout << "\n" << std::endl;
            RCLCPP_INFO(this->get_logger(), "path_resolution set to: %.2f", distanceToObstacle_);
        }

        if(new_diagonalEdges != diagonalEdges_)
        {
            diagonalEdges_ = new_diagonalEdges;

            std::cout << "\n" << std::endl;

            RCLCPP_INFO(this->get_logger(), "diagonalEdges set to: %d", diagonalEdges_);
        }

      
        
    }
    
   
public:
    AStar()
     : Node("Bug_1")
    {
        this->declare_parameter<double>("path_resolution", 0.05);
        this->declare_parameter<int>("diagonalEdges", 3);
        this->declare_parameter<int>("iterations_before_verification", 10);

        distanceToObstacle_ =  static_cast<float>(this->get_parameter("path_resolution").get_parameter_value().get<double>());
        diagonalEdges_ = this->get_parameter("diagonalEdges").get_parameter_value().get<int>();
        iterations_before_verification = this->get_parameter("iterations_before_verification").get_parameter_value().get<int>();

        RCLCPP_INFO(this->get_logger(), "path_resolution is set to: %f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "diagonalEdges is set to: %d", diagonalEdges_);
        RCLCPP_INFO(this->get_logger(), "iterations_before_verification is set to: %d", iterations_before_verification);


        parameterTimer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&AStar::check_parameters, this));

    
        decimals = count_decimals(distanceToObstacle_);

        publisher_created_vertices = this->create_publisher<sensor_msgs::msg::PointCloud2>("/created_vertices", 10);
        publisher_created_vertices1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/created_vertices1", 10);

        subscription_navigable_removed_vertices = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacles_vertices", 10, std::bind(&AStar::callback_removed_navigable_vertices, this, std::placeholders::_1));

        publisher_nav_path_ = this->create_publisher<nav_msgs::msg::Path>("/filtered_path", 10);
        publisher_path_without_filter_ = this->create_publisher<nav_msgs::msg::Path>("/path_without_filter", 10);

        publisher_path_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/path", 10);
        

        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AStar::callback_odom, this, std::placeholders::_1));

        subscription3_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/destinations", 10, std::bind(&AStar::callback_destinations, this, std::placeholders::_1));

     
    
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<AStar>());
    rclcpp::shutdown();
    return 0;
}