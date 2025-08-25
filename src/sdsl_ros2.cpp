#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

class SDSL_ROS2 : public rclcpp::Node {
public:
    SDSL_ROS2() :
        Node("sdsl_ros2") {
    }

private:
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDSL_ROS2>());
    rclcpp::shutdown();
    return 0;
}