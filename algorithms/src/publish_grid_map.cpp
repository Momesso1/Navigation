#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

class MapPublisher : public rclcpp::Node
{
public:
    MapPublisher() : Node("map_publisher")
    {
        this->declare_parameter<std::string>("map_filename", "");
        this->declare_parameter<std::string>("map_pgm_filename", "");

        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MapPublisher::publish_map, this));
    }

private:
    void publish_map()
    {
        if (!grid_) {
            std::string yaml_file, image_file;
            this->get_parameter("map_filename", yaml_file);
            this->get_parameter("map_pgm_filename", image_file);

            if (yaml_file.empty() || image_file.empty()) {
                RCLCPP_WARN(this->get_logger(), "Parâmetros map_filename e map_pgm_filename devem ser definidos");
                return;
            }

            try {
                grid_ = load_map(yaml_file, image_file);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Erro ao carregar mapa: %s", e.what());
                return;
            }
        }

        grid_->header.stamp = this->now();
        pub_->publish(*grid_);
    }

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> load_map(
        const std::string &yaml_file,
        const std::string &image_file_param)
    {
        YAML::Node config = YAML::LoadFile(yaml_file);
        double resolution = config["resolution"].as<double>();
        std::vector<double> origin = config["origin"].as<std::vector<double>>();
        int negate = config["negate"].as<int>();
        double occ_th = config["occupied_thresh"].as<double>();
        double free_th = config["free_thresh"].as<double>();

        // usa o parâmetro passado, não o campo "image:" do yaml
        std::string image_file = image_file_param;

        cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
        if (image.empty())
            throw std::runtime_error("Falha ao carregar imagem: " + image_file);

        auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        grid->header.frame_id = "map";
        grid->info.resolution = resolution;
        grid->info.width = image.cols;
        grid->info.height = image.rows;
        grid->info.origin.position.x = origin[0];
        grid->info.origin.position.y = origin[1];
        grid->info.origin.position.z = 0.0;
        grid->info.origin.orientation.w = 1.0;
        grid->data.resize(grid->info.width * grid->info.height);

        for (int y = 0; y < image.rows; y++) {
            for (int x = 0; x < image.cols; x++) {
                int value = image.at<unsigned char>(y, x);
                if (negate)
                    value = 255 - value;
                double occ = (255 - value) / 255.0;

                int idx = (image.rows - y - 1) * image.cols + x;
                if (occ > occ_th)
                    grid->data[idx] = 100;
                else if (occ < free_th)
                    grid->data[idx] = 0;
                else
                    grid->data[idx] = -1;
            }
        }
        return grid;
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
