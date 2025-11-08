// a_star_2d.cpp
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
#include <vector>
#include <map>
#include <stack>
#include <unordered_map>
#include <unordered_set>
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
// #include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>

using namespace std::chrono_literals;

// Hash for std::pair<float,float>
template <typename T1, typename T2>
struct pair_hash {
    std::size_t operator ()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        // combinação simples
        return h1 ^ (h2 << 1);
    }
};

class DStar2D : public rclcpp::Node {
private:

    struct Vertex {
        int key;
        float x, y;
    };

    struct VertexDijkstra {
        float x, y;
        float orientation_x, orientation_y, orientation_z;
        float orientation_w;
    };

    struct PairHash {
        std::size_t operator()(const std::pair<float, float>& p) const {
            auto h1 = std::hash<float>{}(p.first);
            auto h2 = std::hash<float>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };

    //Publishers.
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_nav_path_;

    //Subscriptions.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_navigable_removed_vertices_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_destinations_;

    //Timers.
    rclcpp::TimerBase::SharedPtr timer_path_;
    rclcpp::TimerBase::SharedPtr timer_visualize_path_;
    rclcpp::TimerBase::SharedPtr parameterTimer_;

    size_t i_ = 0;
    int diagonalEdges_;
    float pose_x_ = 0.0f, pose_y_ = 0.0f;
    float distanceToObstacle_, maximumHeight, minimumHeight;
    float x_upper_bound = 10.0, x_lower_bound = -10.0, y_upper_bound = 10.0, y_lower_bound = -10.0;

    int decimals = 0;

    std::vector<std::pair<float, float>> previousPath; // agora em 2D
    std::vector<VertexDijkstra> verticesDijkstra;
    std::vector<VertexDijkstra> verticesDestino_;

    std::unordered_map<int, std::vector<int>> adjacency_list_;
    std::unordered_set<std::pair<float, float>, PairHash> obstaclesVertices_;
    std::unordered_map<int, Vertex> navigableVerticesMapInteger_;

    inline float roundToMultiple(float value, float multiple, int decimals) {
        if (multiple == 0.0f) return value; // Evita divisão por zero
        float result = std::round(value / multiple) * multiple;
        float factor = std::pow(10.0f, decimals);
        result = std::round(result * factor) / factor;
        return result;
    }

    int countDecimals(float number) {
        float fractional = std::fabs(number - std::floor(number));
        int dec = 0;
        const float epsilon = 1e-9f;
        while (fractional > epsilon && dec < 20) {
            fractional *= 10.0f;
            fractional -= std::floor(fractional);
            dec++;
        }
        return dec;
    }

    // 8 vizinhos em 2D (N, E, S, W + 4 diagonais)
    std::vector<std::array<float, 2>> get_offsets(float distanceToObstacle) {
        return {
            {-distanceToObstacle, 0.0f},
            {0.0f, distanceToObstacle},
            {distanceToObstacle, 0.0f},
            {0.0f, -distanceToObstacle},

            {-distanceToObstacle, distanceToObstacle},
            {distanceToObstacle, distanceToObstacle},
            {distanceToObstacle, -distanceToObstacle},
            {-distanceToObstacle, -distanceToObstacle}
        };
    }

    // D* retornando vetor de pares (x,y)
    std::vector<std::pair<float, float>> runAStar(float start[2], float goal[2]) {
        struct Node {
            std::pair<float, float> parent;
            float g_score = std::numeric_limits<float>::infinity();
            float f_score = std::numeric_limits<float>::infinity();
            bool closed = false;
            bool has_parent = false;
        };

        std::unordered_map<std::pair<float, float>, Node, pair_hash<float,float>> nodes;
        std::unordered_map<std::pair<float, float>, std::vector<std::pair<float, float>>, pair_hash<float,float>> adjacency_list_tuples;

        auto offsets = get_offsets(distanceToObstacle_);

        std::pair<float, float> start_tuple = { start[0], start[1] };
        std::pair<float, float> goal_tuple  = { goal[0],  goal[1]  };

        float new_x = 0.0f, new_y = 0.0f;
        bool findNavigableVertice = false;

        // Encontrar vértices navegáveis próximos ao start (até 2 passos)
        for (int i = 1; i <= 2 && !findNavigableVertice; ++i) {
            for (size_t a = 0; a < offsets.size(); ++a) {
                new_x = roundToMultiple(start_tuple.first + (offsets[a][0] * i), distanceToObstacle_, decimals);
                new_y = roundToMultiple(start_tuple.second + (offsets[a][1] * i), distanceToObstacle_, decimals);
                std::pair<float, float> neighbor = std::make_pair(new_x, new_y );
                if (obstaclesVertices_.find(neighbor) == obstaclesVertices_.end()) {
                    adjacency_list_tuples[start_tuple].push_back(neighbor);
                    findNavigableVertice = true;
                }
            }
        }

        if (!findNavigableVertice) {
            RCLCPP_WARN(this->get_logger(), "The robot is too far from the navigable area.");
            return {};
        }

        bool findNavigableGoalVertice = false;
        for (int i = 1; i <= 2 && !findNavigableGoalVertice; ++i) {
            for (size_t a = 0; a < offsets.size(); ++a) {
                new_x = roundToMultiple(goal_tuple.first + (offsets[a][0] * i), distanceToObstacle_, decimals);
                new_y = roundToMultiple(goal_tuple.second + (offsets[a][1] * i), distanceToObstacle_, decimals);
                std::pair<float, float> neighbor = std::make_pair(new_x, new_y );
                if (obstaclesVertices_.find(neighbor) == obstaclesVertices_.end()) {
                    adjacency_list_tuples[neighbor].push_back(goal_tuple);
                    findNavigableGoalVertice = true;
                }
            }
        }

        if (!findNavigableGoalVertice) {
            RCLCPP_WARN(this->get_logger(), "Destination is too far from the navigable area. Increase navigable area.");
            return {};
        }

        auto heuristic = [](const std::pair<float,float>& a, const std::pair<float,float>& b) {
            float dx = a.first - b.first;
            float dy = a.second - b.second;
            return std::sqrt(dx*dx + dy*dy);
        };

        nodes[start_tuple].g_score = 0.0f;
        nodes[start_tuple].f_score = heuristic(start_tuple, goal_tuple);

        struct PairCompare {
            bool operator()(const std::pair<float, std::pair<float,float>>& a,
                            const std::pair<float, std::pair<float,float>>& b) const {
                return a.first > b.first;
            }
        };

        std::priority_queue<
            std::pair<float, std::pair<float,float>>,
            std::vector<std::pair<float, std::pair<float,float>>>,
            PairCompare
        > open_set;

        open_set.push({nodes[start_tuple].f_score, start_tuple});

        while (!open_set.empty()) {
            auto current_pair = open_set.top();
            open_set.pop();
            auto current = current_pair.second;

            if (nodes[current].closed) continue;
            if (current_pair.first > nodes[current].f_score) continue;

            nodes[current].closed = true;

            // expand neighbors (não expandir start/goal se não necessário)
            if (current != start_tuple && current != goal_tuple) {
                for (size_t a = 0; a < offsets.size(); ++a) {
                    new_x = roundToMultiple(current.first + offsets[a][0], distanceToObstacle_, decimals);
                    new_y = roundToMultiple(current.second + offsets[a][1], distanceToObstacle_, decimals);

                    std::pair<float, float> neighbor = std::make_pair(new_x, new_y );

                    if ((obstaclesVertices_.find(neighbor) == obstaclesVertices_.end()) && (new_x < x_upper_bound && new_x > x_lower_bound && new_y < y_upper_bound && new_y > y_lower_bound)) 
                    {
                        adjacency_list_tuples[current].push_back(neighbor);
                    }
                }
            }

            if (current == goal_tuple) {
                std::vector<std::pair<float,float>> path;
                auto cur = current;
                path.insert(path.begin(), cur);
                while (nodes.find(cur) != nodes.end() && !(cur == start_tuple)) {
                    cur = nodes[cur].parent;
                    path.insert(path.begin(), cur);
                }
                return path;
            }

            for (const auto& neighbor : adjacency_list_tuples[current]) {
                if (nodes.find(neighbor) != nodes.end() && nodes[neighbor].closed) continue;
                float tentative_g_score = nodes[current].g_score + heuristic(current, neighbor);
                if (nodes.find(neighbor) == nodes.end() || tentative_g_score < nodes[neighbor].g_score) {
                    nodes[neighbor].parent = current;
                    nodes[neighbor].has_parent = true;
                    nodes[neighbor].g_score = tentative_g_score;
                    nodes[neighbor].f_score = tentative_g_score + heuristic(neighbor, goal_tuple);
                    open_set.push({nodes[neighbor].f_score, neighbor});
                }
            }
            adjacency_list_tuples.erase(current);
        }

        RCLCPP_WARN(this->get_logger(), "It is not possible to reach the destination.");
        return {};
    }

    // Simplifica caminho (remove waypoints redundantes), agora em 2D
    void storeEdgesInPath(const std::vector<std::pair<float,float>>& originalPath) {
        verticesDijkstra.clear();
        if (originalPath.empty()) return;

        auto start_time_ = std::chrono::high_resolution_clock::now();

        std::vector<std::pair<float,float>> path = originalPath;

        size_t k = 0;
        while (k < path.size() - 1) {
            bool shortcutFound = false;
            for (int i = static_cast<int>(path.size()) - 1; i > static_cast<int>(k); --i) {
                auto A = path[k];
                auto B = path[i];

                float ax = A.first, ay = A.second;
                float bx = B.first, by = B.second;

                float dx = bx - ax, dy = by - ay;
                float distance = std::sqrt(dx*dx + dy*dy);
                if (distance == 0.0f) continue;

                float ux = dx / distance;
                float uy = dy / distance;
                float step = distanceToObstacle_;
                float t = 0.0f;
                bool obstacleFound = false;

                // percorre o segmento e checa obstáculos arredondando para grade
                while (t < distance && !obstacleFound) {
                    float px = ax + t*ux;
                    float py = ay + t*uy;
                    float new_x = roundToMultiple(px, distanceToObstacle_, decimals);
                    float new_y = roundToMultiple(py, distanceToObstacle_, decimals);
                    std::pair<float,float> neighbor = { new_x, new_y };
                    if (obstaclesVertices_.find(neighbor) != obstaclesVertices_.end()) {
                        obstacleFound = true;
                        break;
                    }
                    t += step;
                }

                if (!obstacleFound) {
                    // retira pontos entre k e i
                    path.erase(path.begin() + k + 1, path.begin() + i);
                    shortcutFound = true;
                    break;
                }
            }

            if (shortcutFound) ++k;
            else break;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = end_time - start_time_;
        RCLCPP_INFO(this->get_logger(), "D* filter execution time: %.10f", duration.count());

        // Preencher verticesDijkstra com orientação 2D (yaw -> quaternion z,w)
        for (size_t idx = 0; idx < path.size(); ++idx) {
            VertexDijkstra vertex;
            vertex.x = path[idx].first;
            vertex.y = path[idx].second;
            if (idx < path.size() - 1) {
                auto cur = path[idx];
                auto nxt = path[idx+1];
                float dx = nxt.first - cur.first;
                float dy = nxt.second - cur.second;
                float yaw = std::atan2(dy, dx);
                float half = yaw * 0.5f;
                vertex.orientation_x = 0.0f;
                vertex.orientation_y = 0.0f;
                vertex.orientation_z = std::sin(half);
                vertex.orientation_w = std::cos(half);
            } else {
                vertex.orientation_x = 0.0f;
                vertex.orientation_y = 0.0f;
                vertex.orientation_z = 0.0f;
                vertex.orientation_w = 1.0f;
            }
            verticesDijkstra.push_back(vertex);
        }
    }

    // Publishers
    void publisher_dijkstra() {
        geometry_msgs::msg::PoseArray message;
        message.header.stamp = this->now();
        message.header.frame_id = "world";

        
        for (const auto& v : verticesDijkstra) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = v.x;
            pose.position.y = v.y;
            pose.position.z = 0.0;
            pose.orientation.x = v.orientation_x;
            pose.orientation.y = v.orientation_y;
            pose.orientation.z = v.orientation_z;
            pose.orientation.w = v.orientation_w;
            message.poses.push_back(pose);
        }
        publisher_path_->publish(message);
    }

    void publisher_dijkstra_path() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "world";
        for (const auto& v : verticesDijkstra) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.stamp = this->now();
            ps.header.frame_id = "world";
            ps.pose.position.x = v.x;
            ps.pose.position.y = v.y;
            ps.pose.position.z = 0.0;
            ps.pose.orientation.x = v.orientation_x;
            ps.pose.orientation.y = v.orientation_y;
            ps.pose.orientation.z = v.orientation_z;
            ps.pose.orientation.w = v.orientation_w;
            path_msg.poses.push_back(ps);
        }
        publisher_nav_path_->publish(path_msg);
    }

    // Callbacks
    int l_ = 0;
    float tempo_medio = 0.0f, soma_total = 0.0f;

    void callback_destinations(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
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

        if (verticesDestino_.empty()) return;

        if(obstaclesVertices_.empty()) return;

        // se i_ exceder, volta
        if (i_ >= verticesDestino_.size()) i_ = 0;

        // distância ao destino atual
        float dx = pose_x_ - verticesDestino_[i_].x;
        float dy = pose_y_ - verticesDestino_[i_].y;
        float distanciaAteODestino = std::sqrt(dx*dx + dy*dy);

        if (distanciaAteODestino <= distanceToObstacle_) {
            ++i_;
            if (i_ >= verticesDestino_.size()) i_ = 0;
        }

        float array_inicial[2] = { pose_x_, pose_y_ };
        float array_final[2] = { verticesDestino_[i_].x, verticesDestino_[i_].y };

        auto start_time_ = std::chrono::high_resolution_clock::now();

        // conjunto de obstáculos próximos ao start (até 2 passos) - detecta se previousPath tem interseção
        std::unordered_set<std::pair<float,float>, pair_hash<float,float>> local_obstacles;
        auto offsets = get_offsets(distanceToObstacle_);
        for (int i = 1; i <= 2; ++i) {
            for (size_t a = 0; a < offsets.size(); ++a) {
                float new_x = roundToMultiple(array_inicial[0] + offsets[a][0] * i, distanceToObstacle_, decimals);
                float new_y = roundToMultiple(array_inicial[1] + offsets[a][1] * i, distanceToObstacle_, decimals);
                std::pair<float,float> p = { new_x, new_y };
                if (obstaclesVertices_.find(p) != obstaclesVertices_.end()) {
                    local_obstacles.insert(p);
                }
            }
        }

        bool found = false;
        bool obstacleFound = false;
        std::vector<std::pair<std::pair<float,float>, std::pair<float,float>>> pairs;
        std::vector<int> intervals;
        std::vector<std::pair<float,float>> finalPath;

        std::pair<float,float> origin = {0.0f, 0.0f}, destination = {0.0f, 0.0f};
        intervals.clear();

        if (!previousPath.empty()) {
            previousPath[0] = { array_inicial[0], array_inicial[1] };
            for (size_t m = 1; m + 1 < previousPath.size(); ++m) {
                if (obstaclesVertices_.find(previousPath[m]) != obstaclesVertices_.end() && !obstacleFound && local_obstacles.find(previousPath[m]) == local_obstacles.end() && m > 0) {
                    origin = previousPath[m-1];
                    intervals.push_back(static_cast<int>(m-1));
                    obstacleFound = true;
                    found = true;
                }
                if (obstaclesVertices_.find(previousPath[m]) == obstaclesVertices_.end() && obstacleFound) {
                    destination = previousPath[m];
                    pairs.emplace_back(origin, destination);
                    obstacleFound = false;
                }
            }
            if (!found) {
                // se não foram obstáculos novos, simplifica o previousPath
                storeEdgesInPath(previousPath);
            }
        }

        if (previousPath.empty()) {
            std::vector<std::pair<float,float>> shortestPath = runAStar(array_inicial, array_final);
            previousPath = shortestPath;
            storeEdgesInPath(shortestPath);
        } else if (found) {
            finalPath.clear();
            // adiciona até o primeiro intervalo
            for (int i = 0; i <= intervals[0]; ++i) finalPath.push_back(previousPath[i]);

            for (size_t k = 0; k < pairs.size(); ++k) {
                const auto& pr = pairs[k];
                float s_init[2] = { pr.first.first, pr.first.second };
                float s_end[2]  = { pr.second.first, pr.second.second };
                std::vector<std::pair<float,float>> shortestPath = runAStar(s_init, s_end);

                int startIdx = (finalPath.empty() || finalPath.back() != shortestPath.front()) ? 0 : 1;
                for (size_t i = startIdx; i + 1 < shortestPath.size(); ++i) {
                    finalPath.push_back(shortestPath[i]);
                }

                int nextStart = -1;
                if (k < pairs.size() - 1) {
                    for (size_t i = 0; i < previousPath.size(); ++i) {
                        if (previousPath[i] == pr.second) { nextStart = static_cast<int>(i); break; }
                    }
                    if (nextStart >= 0) {
                        for (int i = nextStart; i <= intervals[k+1]; ++i) finalPath.push_back(previousPath[i]);
                    }
                } 
                else 
                {
                    for (size_t i = 0; i < previousPath.size(); ++i) 
                    {
                        if (previousPath[i] == pr.second) { nextStart = static_cast<int>(i); break; }
                    }
                    if (nextStart >= 0) {
                        for (size_t i = nextStart; i < static_cast<int>(previousPath.size()); ++i) finalPath.push_back(previousPath[i]);
                    }
                }
            }
            previousPath = finalPath;
            storeEdgesInPath(finalPath);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = end_time - start_time_;
        ++l_;
        soma_total += duration.count();
        tempo_medio = soma_total / l_;

        adjacency_list_.clear();

        RCLCPP_INFO(this->get_logger(), "D* execution time: %.10f", duration.count());
        RCLCPP_INFO(this->get_logger(), "Medium D* execution time: %.10f", tempo_medio);
    }

    void callback_removed_navigable_vertices(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
            float x = *iter_x;
            float y = *iter_y;
            auto index = std::make_pair(
                roundToMultiple(x, distanceToObstacle_, decimals),
                roundToMultiple(y, distanceToObstacle_, decimals)
            );
            obstaclesVertices_.insert(index);
        }
    }

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        pose_x_ = msg->pose.pose.position.x;
        pose_y_ = msg->pose.pose.position.y;
    }

    void check_parameters() {
        auto new_distanceToObstacle = static_cast<float>(this->get_parameter("path_resolution").get_parameter_value().get<double>());
        auto new_diagonalEdges = this->get_parameter("diagonalEdges").get_parameter_value().get<int>();
        auto new_minimumHeight = static_cast<float>(this->get_parameter("minimumHeight").get_parameter_value().get<double>());
        auto new_maximumHeight = static_cast<float>(this->get_parameter("maximumHeight").get_parameter_value().get<double>());

        if (new_distanceToObstacle != distanceToObstacle_) {
            distanceToObstacle_ = new_distanceToObstacle;
            RCLCPP_INFO(this->get_logger(), "path_resolution set to: %.2f", distanceToObstacle_);
            decimals = countDecimals(distanceToObstacle_);
        }
        if (new_diagonalEdges != diagonalEdges_) {
            diagonalEdges_ = new_diagonalEdges;
            RCLCPP_INFO(this->get_logger(), "diagonalEdges set to: %d", diagonalEdges_);
        }
        if (new_minimumHeight != minimumHeight) {
            minimumHeight = new_minimumHeight;
            RCLCPP_INFO(this->get_logger(), "minimumHeight set to: %f", minimumHeight);
        }
        if (new_maximumHeight != maximumHeight) {
            maximumHeight = new_maximumHeight;
            RCLCPP_INFO(this->get_logger(), "maximumHeight set to: %f", maximumHeight);
        }
    }

public:
    DStar2D() : Node("D_star_2d") {
        this->declare_parameter<double>("path_resolution", 0.05);
        this->declare_parameter<int>("diagonalEdges", 3);
        this->declare_parameter<double>("minimumHeight", -100);
        this->declare_parameter<double>("maximumHeight", 100);
        this->declare_parameter<double>("x_upper_bound", 10.0);
        this->declare_parameter<double>("x_lower_bound", -10.0);
        this->declare_parameter<double>("y_upper_bound", 10.0);
        this->declare_parameter<double>("y_lower_bound", -10.0);

        distanceToObstacle_ = static_cast<float>(this->get_parameter("path_resolution").get_parameter_value().get<double>());
        diagonalEdges_ = this->get_parameter("diagonalEdges").get_parameter_value().get<int>();
        minimumHeight = static_cast<float>(this->get_parameter("minimumHeight").get_parameter_value().get<double>());
        maximumHeight = static_cast<float>(this->get_parameter("maximumHeight").get_parameter_value().get<double>());
        x_upper_bound = static_cast<float>(this->get_parameter("x_upper_bound").get_parameter_value().get<double>());
        x_lower_bound = static_cast<float>(this->get_parameter("x_lower_bound").get_parameter_value().get<double>());
        y_upper_bound = static_cast<float>(this->get_parameter("y_upper_bound").get_parameter_value().get<double>());
        y_lower_bound = static_cast<float>(this->get_parameter("y_lower_bound").get_parameter_value().get<double>());

        RCLCPP_INFO(this->get_logger(), "path_resolution is set to: %f", distanceToObstacle_);
        RCLCPP_INFO(this->get_logger(), "diagonalEdges is set to: %d", diagonalEdges_);
        RCLCPP_INFO(this->get_logger(), "minimumHeight is set to: %f", minimumHeight);
        RCLCPP_INFO(this->get_logger(), "maximumHeight is set to: %f", maximumHeight);
        RCLCPP_INFO(this->get_logger(), "x_upper_bound is set to: %f", x_upper_bound);
        RCLCPP_INFO(this->get_logger(), "x_lower_bound is set to: %f", x_lower_bound);
        RCLCPP_INFO(this->get_logger(), "y_upper_bound is set to: %f", y_upper_bound);
        RCLCPP_INFO(this->get_logger(), "y_lower_bound is set to: %f", y_lower_bound);

        decimals = countDecimals(distanceToObstacle_);

        parameterTimer_ = this->create_wall_timer(2s, std::bind(&DStar2D::check_parameters, this));

        subscription_navigable_removed_vertices_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacles_vertices", 10,
            std::bind(&DStar2D::callback_removed_navigable_vertices, this, std::placeholders::_1)
        );

        publisher_nav_path_ = this->create_publisher<nav_msgs::msg::Path>("visualize_path", 10);
        timer_visualize_path_ = this->create_wall_timer(1ms, std::bind(&DStar2D::publisher_dijkstra_path, this));

        publisher_path_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/path", 10);
        timer_path_ = this->create_wall_timer(1ms, std::bind(&DStar2D::publisher_dijkstra, this));

        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DStar2D::callback_odom, this, std::placeholders::_1));

        subscription_destinations_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/destinations", 10, std::bind(&DStar2D::callback_destinations, this, std::placeholders::_1));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DStar2D>());
    rclcpp::shutdown();
    return 0;
}
