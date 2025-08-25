#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#define SDSL_CPP_ONLY
#include <sdsl.hpp>
#include <environments/env_R3_pcd.hpp>
#include <CGAL/Simple_cartesian.h>
using Kernel = CGAL::Simple_cartesian<double>;
using FT = Kernel::FT;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class SDSL_ROS2 : public rclcpp::Node {
public:
    SDSL_ROS2() :
        Node("sdsl_ros2") {
        mapSubscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&SDSL_ROS2::mapCallback, this, std::placeholders::_1));
    }

private:
    // Subscriptions and publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscription_;

    // SDSL structures
    sdsl::Env_R3_PCD<Kernel> environment_;

    // ---------------------------------------------------------------------------------------------
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received map with size: %d x %d", msg->info.width, msg->info.height);

        std::vector<Kernel::Point_3> points;
        for (int y = 0; y < msg->info.height; ++y)
        for (int x = 0; x < msg->info.width; ++x) {
            int idx = x + y * msg->info.width;
            int8_t cell = msg->data[idx];
            if (cell < 0) continue;
            if (cell > 20) continue; // TODO: Move the hardcoded threshold to a parameter
            double wx = msg->info.origin.position.x + (x + 0.5) * msg->info.resolution;
            double wy = msg->info.origin.position.y + (y + 0.5) * msg->info.resolution;
            Kernel::Point_3 pt(wx, wy, 0.0);
            points.push_back(pt);
        }
        environment_ = sdsl::Env_R3_PCD<Kernel>(points);
        RCLCPP_INFO(this->get_logger(), "Environment constructed with %zu points", points.size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDSL_ROS2>());
    rclcpp::shutdown();
    return 0;
}