#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <mutex>
#include <vector>

class DijkstraController3D : public rclcpp::Node
{
public:
    DijkstraController3D() : Node("dijkstra_controller_3d"), 
                            current_waypoint_idx_(0),
                            path_received_(false),
                            executing_path_(false)
    {
        // Publishers e Subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DijkstraController3D::odom_callback, this, std::placeholders::_1));
            
        vertex_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/path", 10, std::bind(&DijkstraController3D::vertex_callback, this, std::placeholders::_1));

        // Timer para controle contínuo
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz
            std::bind(&DijkstraController3D::control_loop, this));

        // Parâmetros do controlador
        linear_speed_ = 1.0;  // Aumentei a velocidade
        angular_speed_ = 1.5; // Aumentei a velocidade angular
        waypoint_tolerance_ = 0.5; // Aumentei a tolerância
        angle_tolerance_ = 0.2; // Relaxei a tolerância angular
        
        RCLCPP_INFO(this->get_logger(), "DijkstraController3D iniciado!");
    }

private:
    // Publishers e Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr vertex_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Estado do robô
    geometry_msgs::msg::Pose current_pose_;
    bool pose_initialized_ = false;
    
    // Estado do caminho
    std::vector<geometry_msgs::msg::Pose> current_path_;
    std::mutex path_mutex_;
    size_t current_waypoint_idx_;
    bool path_received_;
    bool executing_path_;
    
    // Parâmetros do controlador
    double linear_speed_;
    double angular_speed_;
    double waypoint_tolerance_;
    double angle_tolerance_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        pose_initialized_ = true;
    }

    void vertex_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        
        // Sempre aceita um novo caminho
        current_path_.clear();
        current_path_.reserve(msg->poses.size());
        
        for (const auto& pose : msg->poses) {
            current_path_.push_back(pose);
        }
        
        if (!current_path_.empty()) {
            // Reset para o início do novo caminho
            current_waypoint_idx_ = 0;
            path_received_ = true;
            executing_path_ = true;
            
            RCLCPP_INFO(this->get_logger(), 
                "Novo caminho recebido com %zu pontos. Reiniciando navegação.", 
                current_path_.size());
        }
    }

    void control_loop()
    {
        if (!pose_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Aguardando dados de odometria...");
            publish_zero_velocity();
            return;
        }
        
        if (!path_received_ || !executing_path_) {
            // Para o robô se não há caminho válido
            publish_zero_velocity();
            return;
        }

        std::lock_guard<std::mutex> lock(path_mutex_);
        
        if (current_path_.empty() || current_waypoint_idx_ >= current_path_.size()) {
            // Caminho finalizado
            publish_zero_velocity();
            executing_path_ = false;
            RCLCPP_INFO(this->get_logger(), "Caminho finalizado!");
            return;
        }

        // Pega o waypoint atual
        const auto& target_pose = current_path_[current_waypoint_idx_];
        
        // Calcula distância para o target
        double dx = target_pose.position.x - current_pose_.position.x;
        double dy = target_pose.position.y - current_pose_.position.y;
        double distance = sqrt(dx * dx + dy * dy);
        
        // Log da posição atual e target
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Pos atual: (%.2f, %.2f) -> Target: (%.2f, %.2f), Dist: %.2f, WP: %zu/%zu",
            current_pose_.position.x, current_pose_.position.y,
            target_pose.position.x, target_pose.position.y,
            distance, current_waypoint_idx_ + 1, current_path_.size());
        
        // Verifica se chegou no waypoint atual
        if (distance < waypoint_tolerance_) {
            current_waypoint_idx_++;
            RCLCPP_INFO(this->get_logger(), 
                "Waypoint %zu alcançado. Próximo: %zu/%zu", 
                current_waypoint_idx_ - 1, current_waypoint_idx_, current_path_.size());
            return;
        }

        // Calcula e publica controle
        geometry_msgs::msg::Twist cmd_vel = calculate_control(target_pose);
        
        // Log dos comandos
        RCLCPP_DEBUG(this->get_logger(), "Cmd: linear=%.2f, angular=%.2f", 
                    cmd_vel.linear.x, cmd_vel.angular.z);
        
        cmd_vel_pub_->publish(cmd_vel);
    }

    geometry_msgs::msg::Twist calculate_control(const geometry_msgs::msg::Pose& target_pose)
    {
        geometry_msgs::msg::Twist cmd_vel;
        
        // Calcula direção para o target
        double dx = target_pose.position.x - current_pose_.position.x;
        double dy = target_pose.position.y - current_pose_.position.y;
        double distance = sqrt(dx * dx + dy * dy);
        double target_yaw = atan2(dy, dx);
        
        // Pega orientação atual
        tf2::Quaternion current_quat(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        double current_yaw = get_yaw_from_quaternion(current_quat);
        
        // Calcula erro angular
        double angle_error = normalize_angle(target_yaw - current_yaw);
        
        // Debug info
        RCLCPP_DEBUG(this->get_logger(), 
            "Target: (%.2f, %.2f), Current: (%.2f, %.2f), Distance: %.2f, Angle Error: %.2f", 
            target_pose.position.x, target_pose.position.y,
            current_pose_.position.x, current_pose_.position.y,
            distance, angle_error);
        
        // Sempre move para frente com velocidade proporcional à distância
        cmd_vel.linear.x = std::min(linear_speed_, distance * 0.5);
        cmd_vel.linear.x = std::max(cmd_vel.linear.x, 0.1); // Velocidade mínima
        
        // Controle angular proporcional
        double angular_gain = 2.0;
        cmd_vel.angular.z = std::max(-angular_speed_, 
                           std::min(angular_speed_, angle_error * angular_gain));
        
        // Se o erro angular é muito grande, reduz velocidade linear
        if (fabs(angle_error) > M_PI/3) { // 60 graus
            cmd_vel.linear.x *= 0.3;
        } else if (fabs(angle_error) > M_PI/6) { // 30 graus
            cmd_vel.linear.x *= 0.7;
        }
        
        return cmd_vel;
    }

    void publish_zero_velocity()
    {
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.linear.y = 0.0;
        stop_cmd.linear.z = 0.0;
        stop_cmd.angular.x = 0.0;
        stop_cmd.angular.y = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
    }

    double get_yaw_from_quaternion(const tf2::Quaternion& q)
    {
        // Converte quaternion para yaw (rotação em Z)
        double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        return atan2(siny_cosp, cosy_cosp);
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DijkstraController3D>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}