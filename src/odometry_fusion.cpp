#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class SDSL_OdometryFusion : public rclcpp::Node {
public:
    SDSL_OdometryFusion() :
        Node("odometry_fusion") {

    }

private:
    // Subscriptions and publishes
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sdslRawSubscriber_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDSL_OdometryFusion>());
    rclcpp::shutdown();
    return 0;
}