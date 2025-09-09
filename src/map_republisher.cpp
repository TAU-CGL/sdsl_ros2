#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

class MapRepublisher : public rclcpp::Node {
public:
    MapRepublisher() : Node("map_republisher") {
        // Subscribe to map with transient_local QoS to get latched data
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        
        mapSubscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map_original", qos, std::bind(&MapRepublisher::mapCallback, this, std::placeholders::_1));
        
        mapPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&MapRepublisher::publishMap, this));
        
        RCLCPP_INFO(this->get_logger(), "MapRepublisher node started - waiting for map data");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latestMap_ = msg;
        RCLCPP_INFO(this->get_logger(), "Received map: %dx%d", msg->info.width, msg->info.height);
    }
    
    void publishMap() {
        if (latestMap_) {
            auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>(*latestMap_);
            map->header.stamp = this->now();
            mapPublisher_->publish(*map);
        }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latestMap_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapRepublisher>());
    rclcpp::shutdown();
    return 0;
}