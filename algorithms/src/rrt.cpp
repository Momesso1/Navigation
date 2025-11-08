#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <cmath>
#include <random>
#include <memory>
#include <algorithm>
#include <unordered_set>
#include <utility>
#include <chrono>

struct PairHash {
    std::size_t operator()(const std::pair<float, float>& p) const noexcept {
        auto h1 = std::hash<float>{}(p.first);
        auto h2 = std::hash<float>{}(p.second);
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1<<6) + (h1>>2));
    }
};

struct RRTNode2D {
    float x;
    float y;
    RRTNode2D* parent;
    RRTNode2D(float _x, float _y) : x(_x), y(_y), parent(nullptr) {}
};

class RRT2DNode : public rclcpp::Node {
public:
    RRT2DNode()
    : Node("rrt_2d_node"), gen_(rd_()) {
        this->declare_parameter<double>("path_resolution", 0.05);
        this->declare_parameter<int>("max_iterations", 20000);
        this->declare_parameter<double>("goal_sample_rate", 0.05);
        this->declare_parameter<double>("goal_radius", 0.5);
        this->declare_parameter<std::string>("frame_id", "world");

        this->declare_parameter<double>("x_lower_bound", -10.0);
        this->declare_parameter<double>("x_upper_bound", 10.0);
        this->declare_parameter<double>("y_lower_bound", -10.0);
        this->declare_parameter<double>("y_upper_bound", 10.0);

        path_resolution_ = this->get_parameter("path_resolution").as_double();
        max_iterations_ = this->get_parameter("max_iterations").as_int();
        goal_sample_rate_ = this->get_parameter("goal_sample_rate").as_double();
        goal_radius_ = this->get_parameter("goal_radius").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();

        x_lower_ = this->get_parameter("x_lower_bound").as_double();
        x_upper_ = this->get_parameter("x_upper_bound").as_double();
        y_lower_ = this->get_parameter("y_lower_bound").as_double();
        y_upper_ = this->get_parameter("y_upper_bound").as_double();

        decimals_ = countDecimals(static_cast<float>(path_resolution_));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RRT2DNode::odomCallback, this, std::placeholders::_1));
        goals_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/destinations", 10, std::bind(&RRT2DNode::goalsCallback, this, std::placeholders::_1));
        obstacles_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacles_vertices", 10, std::bind(&RRT2DNode::obstaclesCallback, this, std::placeholders::_1));

        // Tópicos atualizados conforme solicitado
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/visualize_path", 10);
        posearray_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/path", 10);
        tree_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt_tree", 10);
    }

private:
    double path_resolution_{0.2};
    int max_iterations_{2000};
    double goal_sample_rate_{0.05};
    double goal_radius_{0.5};
    std::string frame_id_{"map"};
    int decimals_{0};

    double x_lower_, x_upper_, y_lower_, y_upper_;

    nav_msgs::msg::Odometry current_odom_;
    bool have_odom_{false};
    std::vector<std::pair<float,float>> goals_;
    std::vector<std::unique_ptr<RRTNode2D>> nodes_;
    std::unordered_set<std::pair<float,float>, PairHash> obstacles_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goals_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posearray_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;

    std::random_device rd_;
    std::mt19937 gen_;

    inline float roundToMultiple(float value, float multiple, int decimals) {
        if (multiple == 0.0f) return value;
        float result = std::round(value / multiple) * multiple;
        float factor = std::pow(10.0f, decimals);
        result = std::round(result * factor) / factor;
        return result;
    }

    int countDecimals(float number) {
        float fractional = std::fabs(number - std::floor(number));
        int dec = 0;
        const float eps = 1e-9f;
        while (fractional > eps && dec < 20) {
            fractional *= 10.0f;
            fractional -= std::floor(fractional);
            dec++;
        }
        return dec;
    }

    bool withinBounds(float x, float y) {
        return (x >= x_lower_ && x <= x_upper_ && y >= y_lower_ && y <= y_upper_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = *msg;
        have_odom_ = true;
    }

    void goalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        goals_.clear();
        for (auto &p : msg->poses)
            goals_.emplace_back(p.position.x, p.position.y);
        if (have_odom_) runRRT();
    }

    void obstaclesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        obstacles_.clear();
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        for (; it_x != it_x.end(); ++it_x, ++it_y) {
            auto p = std::make_pair(
                roundToMultiple(*it_x, path_resolution_, decimals_),
                roundToMultiple(*it_y, path_resolution_, decimals_)
            );
            obstacles_.insert(p);
        }
    }

    bool collisionFree(float x, float y) {
        if (!withinBounds(x, y)) return false;
        auto p = std::make_pair(
            roundToMultiple(x, path_resolution_, decimals_),
            roundToMultiple(y, path_resolution_, decimals_)
        );
        return obstacles_.find(p) == obstacles_.end();
    }

    geometry_msgs::msg::Point sampleRandomPoint(const std::pair<float,float>& goal) {
        geometry_msgs::msg::Point p;
        std::uniform_real_distribution<double> uni(0.0, 1.0);

        if (uni(gen_) < goal_sample_rate_) {
            auto gp = std::make_pair(
                roundToMultiple(goal.first, path_resolution_, decimals_),
                roundToMultiple(goal.second, path_resolution_, decimals_)
            );
            // 🚫 impede amostragem se o goal estiver em obstáculo
            if (obstacles_.find(gp) == obstacles_.end() && withinBounds(goal.first, goal.second)) {
                p.x = goal.first; p.y = goal.second; p.z = 0.0;
                return p;
            }
        }

        std::uniform_real_distribution<double> xdist(x_lower_, x_upper_);
        std::uniform_real_distribution<double> ydist(y_lower_, y_upper_);

        while (true) {
            float rx = roundToMultiple(static_cast<float>(xdist(gen_)), path_resolution_, decimals_);
            float ry = roundToMultiple(static_cast<float>(ydist(gen_)), path_resolution_, decimals_);
            if (collisionFree(rx, ry)) {
                p.x = rx; p.y = ry; p.z = 0.0;
                return p;
            }
        }
    }

    RRTNode2D* nearestNode(float x, float y) {
        RRTNode2D* nearest = nullptr;
        float min_dist = std::numeric_limits<float>::max();
        for (auto& n : nodes_) {
            float d = std::hypot(n->x - x, n->y - y);
            if (d < min_dist) {
                min_dist = d;
                nearest = n.get();
            }
        }
        return nearest;
    }

    /**
     * @brief Verifica se um segmento de linha reto entre dois pontos colide com obstáculos.
     * @param from_x Coordenada X inicial.
     * @param from_y Coordenada Y inicial.
     * @param to_x Coordenada X final.
     * @param to_y Coordenada Y final.
     * @return true se houver colisão, false caso contrário.
     */
    bool checkSegmentCollision(float from_x, float from_y, float to_x, float to_y) {
        float dx = to_x - from_x;
        float dy = to_y - from_y;
        float dist = std::hypot(dx, dy);
        if (dist == 0.0f) return false; // Sem movimento, sem colisão

        // Garante que verificamos pelo menos na resolução do caminho
        int steps = static_cast<int>(dist / path_resolution_);
        if (steps == 0) steps = 1;

        for (int i = 1; i <= steps; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(steps);
            float x = from_x + dx * t;
            float y = from_y + dy * t;
            if (!collisionFree(x, y)) {
                return true; // Colisão encontrada
            }
        }
        // Verifica também o ponto final
        if (!collisionFree(to_x, to_y)) return true;

        return false; // Sem colisão
    }

    bool segmentCollision(RRTNode2D* from, float to_x, float to_y) {
        // Agora usa a função auxiliar
        return checkSegmentCollision(from->x, from->y, to_x, to_y);
    }


    /**
     * @brief Simplifica um caminho 2D removendo waypoints redundantes.
     * Tenta criar "atalhos" retos entre os nós do caminho, pulando
     * nós intermediários se o atalho for livre de colisão.
     * @param originalPath O caminho bruto gerado pelo RRT.
     * @return Um novo vetor de pares (x, y) representando o caminho simplificado.
     */
    std::vector<std::pair<float,float>> storeEdgesInPath(const std::vector<std::pair<float,float>>& originalPath) {
        if (originalPath.size() < 3) {
            return originalPath; // Não há pontos suficientes para simplificar
        }

        auto start_time_ = std::chrono::high_resolution_clock::now();
        std::vector<std::pair<float,float>> path = originalPath;
        
        size_t k = 0; // k é o índice do ponto de início do "atalho"

        // Itera enquanto k não for o penúltimo ponto
        while (k < path.size() - 2) {
            bool shortcutFound = false;
            // Tenta conectar 'k' ao ponto 'i' mais distante possível
            // (i > k + 1, ou seja, pulando pelo menos um ponto)
            for (int i = static_cast<int>(path.size()) - 1; i > static_cast<int>(k) + 1; --i) {
                
                // Verifica se o segmento reto (k -> i) está livre de colisão
                if (!checkSegmentCollision(path[k].first, path[k].second, path[i].first, path[i].second)) {
                    // Atalho válido! Remove todos os pontos entre k e i
                    // path.begin() + k + 1 é o primeiro ponto a ser removido
                    // path.begin() + i é o ponto *depois* do último a ser removido
                    path.erase(path.begin() + k + 1, path.begin() + i);
                    shortcutFound = true;
                    break; // Sai do loop 'i'
                }
            }

            if (!shortcutFound) {
                // Não foi encontrado nenhum atalho a partir de 'k'
                // Avança 'k' para o próximo ponto
                k++;
            }
            // Se um atalho foi encontrado, 'k' permanece o mesmo
            // para tentar encontrar um novo atalho a partir dele no caminho agora mais curto
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = end_time - start_time_;
        RCLCPP_INFO(this->get_logger(), "Path simplification time: %.10f", duration.count());

        return path; // Retorna o caminho simplificado
    }

    void publishTree() {
        visualization_msgs::msg::MarkerArray tree;
        visualization_msgs::msg::Marker points, lines;
        points.header.frame_id = frame_id_;
        lines.header.frame_id = frame_id_;
        points.header.stamp = lines.header.stamp = now();
        points.ns = lines.ns = "rrt_tree";
        points.id = 0; lines.id = 1;
        points.type = visualization_msgs::msg::Marker::POINTS;
        lines.type = visualization_msgs::msg::Marker::LINE_LIST;
        points.scale.x = points.scale.y = 0.03;
        lines.scale.x = 0.01;
        points.color.r = 0.0; points.color.g = 1.0; points.color.b = 0.0; points.color.a = 1.0;
        lines.color.r = 0.0; lines.color.g = 0.0; lines.color.b = 1.0; lines.color.a = 1.0;

        for (auto &n : nodes_) {
            geometry_msgs::msg::Point pt;
            pt.x = n->x; pt.y = n->y; pt.z = 0.0;
            points.points.push_back(pt);
            if (n->parent) {
                geometry_msgs::msg::Point p1, p2;
                p1.x = n->x; p1.y = n->y;
                p2.x = n->parent->x; p2.y = n->parent->y;
                lines.points.push_back(p1);
                lines.points.push_back(p2);
            }
        }

        tree.markers.push_back(points);
        tree.markers.push_back(lines);
        tree_pub_->publish(tree);
    }

    void runRRT() {
        if (goals_.empty()) return;

        auto goal = goals_.front();
        nodes_.clear();

        auto start = std::make_unique<RRTNode2D>(
            current_odom_.pose.pose.position.x,
            current_odom_.pose.pose.position.y
        );
        nodes_.push_back(std::move(start));

        RRTNode2D* goal_node = nullptr;
        bool reached = false;

        for (int i = 0; i < max_iterations_; ++i) {
            auto rand_point = sampleRandomPoint(goal);
            RRTNode2D* nearest = nearestNode(rand_point.x, rand_point.y);
            if (!nearest) continue;

            float theta = std::atan2(rand_point.y - nearest->y, rand_point.x - nearest->x);
            float new_x = nearest->x + path_resolution_ * std::cos(theta);
            float new_y = nearest->y + path_resolution_ * std::sin(theta);

            // 🚫 não adiciona se novo ponto for obstáculo
            auto new_pair = std::make_pair(
                roundToMultiple(new_x, path_resolution_, decimals_),
                roundToMultiple(new_y, path_resolution_, decimals_)
            );
            if (obstacles_.find(new_pair) != obstacles_.end()) continue;

            if (!collisionFree(new_x, new_y)) continue;
            if (segmentCollision(nearest, new_x, new_y)) continue;

            auto new_node = std::make_unique<RRTNode2D>(new_x, new_y);
            new_node->parent = nearest;
            nodes_.push_back(std::move(new_node));

            float dist_to_goal = std::hypot(new_x - goal.first, new_y - goal.second);
            if (dist_to_goal < goal_radius_) {
                reached = true;
                goal_node = nodes_.back().get();
                break;
            }
        }

        publishTree();

        if (!reached) {
            RCLCPP_WARN(this->get_logger(), "Goal not reached.");
            return;
        }

        // 1. Obter o caminho bruto
        std::vector<std::pair<float,float>> raw_path;
        for (RRTNode2D* n = goal_node; n != nullptr; n = n->parent)
            raw_path.emplace_back(n->x, n->y);
        std::reverse(raw_path.begin(), raw_path.end());

        // 2. Chamar a função de simplificação
        auto simplified_path = storeEdgesInPath(raw_path);
        RCLCPP_INFO(this->get_logger(), "Path simplified from %zu to %zu points.", raw_path.size(), simplified_path.size());


        // 3. Publicar o caminho simplificado
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.frame_id = frame_id_;
        pose_array.header.stamp = now();

        nav_msgs::msg::Path path_msg;
        path_msg.header = pose_array.header;

        // Itera sobre o simplified_path
        for (auto& p : simplified_path) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.first;
            pose.position.y = p.second;
            // A orientação (yaw) pode ser calculada aqui se necessário
            // Por enquanto, deixamos como padrão (sem rotação)
            pose.orientation.w = 1.0; 
            pose_array.poses.push_back(pose);

            geometry_msgs::msg::PoseStamped ps;
            ps.header = pose_array.header; // Define o header para cada PoseStamped
            ps.pose = pose;
            path_msg.poses.push_back(ps);
        }

        posearray_pub_->publish(pose_array); // Publica em /path
        path_pub_->publish(path_msg);         // Publica em /visualize_path
        RCLCPP_INFO(this->get_logger(), "RRT path published (%zu points).", simplified_path.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RRT2DNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}