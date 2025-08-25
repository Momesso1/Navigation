#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>

class SimpleMapPublisher : public rclcpp::Node
{
public:
    SimpleMapPublisher() : Node("simple_map_publisher")
    {
        this->declare_parameter<std::string>("yaml_filename", "map.yaml");
        std::string yaml_file = this->get_parameter("yaml_filename").as_string();

        if (!loadMap(yaml_file)) {
            RCLCPP_FATAL(this->get_logger(), "Falha ao carregar mapa %s", yaml_file.c_str());
            rclcpp::shutdown();
            return;
        }

        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
        pub_->publish(map_msg_);
        RCLCPP_INFO(this->get_logger(), "Mapa publicado no tópico /map");
    }

private:
    bool loadMap(const std::string & yaml_filename)
    {
        YAML::Node config = YAML::LoadFile(yaml_filename);

        std::string image_file = config["image"].as<std::string>();
        double resolution = config["resolution"].as<double>();
        std::vector<double> origin = config["origin"].as<std::vector<double>>();
        double occupied_thresh = config["occupied_thresh"].as<double>();
        double free_thresh = config["free_thresh"].as<double>();

        cv::Mat image = cv::imread(image_file, cv::IMREAD_UNCHANGED);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao abrir imagem %s", image_file.c_str());
            return false;
        }

        map_msg_.header.frame_id = "map";
        map_msg_.info.resolution = resolution;
        map_msg_.info.width = image.cols;
        map_msg_.info.height = image.rows;
        map_msg_.info.origin.position.x = origin[0];
        map_msg_.info.origin.position.y = origin[1];
        map_msg_.info.origin.orientation.w = 1.0;
        map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height);

        for (int y = 0; y < image.rows; ++y) {
            for (int x = 0; x < image.cols; ++x) {
                uint8_t pixel = image.at<uint8_t>(image.rows - y - 1, x);
                int index = x + y * map_msg_.info.width;
                double occ = (255 - pixel) / 255.0;
                if (occ > occupied_thresh)
                    map_msg_.data[index] = 100;
                else if (occ < free_thresh)
                    map_msg_.data[index] = 0;
                else
                    map_msg_.data[index] = -1;
            }
        }
        return true;
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    nav_msgs::msg::OccupancyGrid map_msg_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleMapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
