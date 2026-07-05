#include <map>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#define SDSL_CPP_ONLY
#include <sdsl/sdsl.hpp>
#include <sdsl/environments/env_pgm.hpp>
#include <sdsl/predicates/pred_forward_2d.hpp>
using Kernel = CGAL::Simple_cartesian<double>;
using FT = Kernel::FT;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sdsl_ros2/msg/point_cloud_with_transform.hpp"

using namespace std::chrono_literals;

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using LaserScan = sensor_msgs::msg::LaserScan;
using PointCloudWithTransform = sdsl_ros2::msg::PointCloudWithTransform;

class SDSL_ROS2 : public rclcpp::Node {
public:
    SDSL_ROS2() : Node("sdsl_ros2"), environmentInitialized_(false) {
        using std::placeholders::_1;
        mapSubscription_ = this->create_subscription<OccupancyGrid>(
            "map", 10, std::bind(&SDSL_ROS2::mapCallback, this, _1)); 
        sdsSubscription_ = this->create_subscription<LaserScan<(
            "sds", 10, std::bind(&SDSL_ROS2::sdsCallback, this, _1));
        pcWithTransformPublisher_ = this->create_publisher<PointCloudWithTransform>("sdsl_raw", 10);

        // 'Subscribe' to the pose of the robot
        tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        RCLCPP_INFO(this->get_logger(), "SDSL_ROS2 node has been started.");
	}

private:
    rclcpp::Subscription<OccupancyGrid>::SharedPtr mapSubscription_;
    rclcpp::Subscription<LaserScan>::SharedPtr sdsSubscription_;
    rclcpp::Publisher<PointCloudWithTransform>::SharedPtr pcWithTransformPublisher_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    sdsl::Env_PGM<3> environment_;
    bool environmentInitialized_;


    // --------------------------------------------------------------------------
    void mapCallback(const OccupancyGrid::SharedPtr msg) {
        printMapStats(msg);
        loadMapFromOccupancyGridMessage(msg);
        RCLCPP_INFO(this->get_logger(), "Environment constructed.");
    }

    void sdsCallback(const LaserScan::SharedPtr msg) {

    }

    void loadMapFromOccupancyGridMessage(const OccupancyGrid::SharedPtr msg) {
        int width = msg->info.width;
        int height = msg->info.height;
        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;
        std::vector<uint8_t> pixels = convertOccupancyGridToPGM(msg);
        environment_.load(pixels.data(), width, height, resolution, origin_x, origin_y);
        environmentInitialized_ = true;
    }

    void printMapStats(const OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received map with size: %d x %d", msg->info.width, msg->info.height);
        std::map<int, int> histogram;
        for (const auto& cell : msg->data) {
            histogram[cell]++;
        }
        for (const auto& [value, count] : histogram) {
            RCLCPP_INFO(this->get_logger(), "Value: %d, Count: %d", value, count);
        }
    }

    std::vector<uint8_t> convertOccupancyGridToPGM(const OccupancyGrid::SharedPtr msg) {
        // Unfortunately, we need to remap the ROS values from -1 U [0,100] to [0, 255]
        // (To match the PGM file format which is also expected in the SDSL library)
        int width = msg->info.width;
        int height = msg->info.height;
        std::vector<uint8_t> pixels(width * height);
        for (int row = 0; row < height; ++row) {
            int srcRow = height - 1 - row; // Flip as images are stored differently
            for (int col = 0; col < width; ++col) {
                int8_t p = msg->data[srcRow * width + col]; // -1 U [0,100]
                pixels[row * width + col] = rescaleOccupancyGridValueToPGM(p);
            }
        }
        return pixels;
    }

    uint8_t rescaleOccupancyGridValueToPGM(int8_t val) {
        if (val < 0) return uint8_t(205); // Return some mid-gray for unknown
        return uint8_t(255.0 * (1.0 - val / 100.0));
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDSL_ROS2>());
    rclcpp::shutdown();
    return 0;
}
