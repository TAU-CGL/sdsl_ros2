#include <map>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#define SDSL_CPP_ONLY
#include <sdsl.hpp>
#include <environments/env_R3_pcd.hpp>
#include <predicates/predicate_static.hpp>
#include <predicates/predicate_dynamic.hpp>
#include <CGAL/Simple_cartesian.h>
using Kernel = CGAL::Simple_cartesian<double>;
using FT = Kernel::FT;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class SDSL_ROS2 : public rclcpp::Node {
public:
    SDSL_ROS2() :
        Node("sdsl_ros2"), environmentInitialized_(false) {
        mapSubscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&SDSL_ROS2::mapCallback, this, std::placeholders::_1));
        sdsSubscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "sds", 10, std::bind(&SDSL_ROS2::sdsCallback, this, std::placeholders::_1));
        pcPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdsl_raw", 10);
        RCLCPP_INFO(this->get_logger(), "SDSL_ROS2 node has been started.");
    }

private:
    // Subscriptions and publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sdsSubscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcPublisher_;

    // SDSL structures
    sdsl::Env_R3_PCD<Kernel> environment_;
    bool environmentInitialized_;

    // ---------------------------------------------------------------------------------------------
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received map with size: %d x %d", msg->info.width, msg->info.height);

        // Print a histogram of values of msg->data
        std::map<int, int> histogram;
        for (const auto& cell : msg->data) {
            histogram[cell]++;
        }
        for (const auto& [value, count] : histogram) {
            RCLCPP_INFO(this->get_logger(), "Value: %d, Count: %d", value, count);
        }

        std::vector<Kernel::Point_3> points;
        for (int y = 0; y < msg->info.height; ++y)
        for (int x = 0; x < msg->info.width; ++x) {
            int idx = x + y * msg->info.width;
            int8_t cell = msg->data[idx];
            if (cell <= 0) continue; // Treat only occupied cells
            double wx = msg->info.origin.position.x + (x + 0.5) * msg->info.resolution;
            double wy = msg->info.origin.position.y + (y + 0.5) * msg->info.resolution;
            Kernel::Point_3 pt(wx, wy, 0.0);
            points.push_back(pt);
        }
        environment_ = sdsl::Env_R3_PCD<Kernel>(points);
        environmentInitialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Environment constructed with %zu points", points.size());
    }

    void sdsCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received laser scan with %zu ranges", msg->ranges.size());

        if (!environmentInitialized_) return;

        std::vector<sdsl::R3xS2<FT>> gs;
        std::vector<FT> ds;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max || std::isnan(range)) {
                continue; // Ignore invalid ranges
            }
            double angle = msg->angle_min + i * msg->angle_increment;
            sdsl::R3xS2<FT> g(
                0.0, 0.0, 0.0, // Sensor at origin
                std::cos(angle), std::sin(angle), 0.0 // Direction in XY plane
            );
            gs.push_back(g);
            ds.push_back(range);
        }

        // sdsl::Predicate_Static<sdsl::R3xS1<FT>, sdsl::R3xS2<FT>, FT, sdsl::Env_R3_PCD<Kernel>> predicate;
        sdsl::Predicate_Dynamic_Naive_Fast<sdsl::R3xS1<FT>, sdsl::R3xS2<FT>, FT, sdsl::Env_R3_PCD<Kernel>> predicate(ds.size(), ds.size()-2);
        FT errorBound = 0.05; // TODO: Move to parameter
        int recursionDepth = 8;    // TODO: Move to parameter
        
        // Localize and report algorithm time
        auto start = std::chrono::steady_clock::now();
        auto result = sdsl::localize<sdsl::R3xS1<FT>, sdsl::R3xS2<FT>, FT, sdsl::Env_R3_PCD<Kernel>>(
            environment_, gs, ds, errorBound, recursionDepth, predicate
        );
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        RCLCPP_INFO(this->get_logger(), "Localization result: %d", result.size());
        RCLCPP_INFO(this->get_logger(), "Localization took: %.2f seconds", elapsed.count());

        // Create a 2D pointcloud that is the midpoint of each voxel
        sensor_msgs::msg::PointCloud2 pointcloud_msg;
        // pointcloud_msg.header = msg->header; // Use same timestamp and frame
        pointcloud_msg.header.stamp = this->now();
        pointcloud_msg.header.frame_id = "map";
        pointcloud_msg.height = 1; // Unordered point cloud
        pointcloud_msg.width = result.size();
        pointcloud_msg.is_dense = true;
        
        // Define point cloud fields (x, y, z)
        sensor_msgs::msg::PointField field_x, field_y, field_z;
        field_x.name = "x";
        field_x.offset = 0;
        field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_x.count = 1;
        
        field_y.name = "y";
        field_y.offset = 4;
        field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_y.count = 1;

        field_z.name = "z";
        field_z.offset = 8;
        field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_z.count = 1;
                
        pointcloud_msg.fields = {field_x, field_y, field_z};
        pointcloud_msg.point_step = 3 * 4; // 3 floats * 4 bytes each
        pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width;
        
        // Populate point cloud data
        pointcloud_msg.data.resize(pointcloud_msg.row_step);
        float* data_ptr = reinterpret_cast<float*>(pointcloud_msg.data.data());
        
        for (size_t i = 0; i < result.size(); ++i) {
            // Get midpoint of voxel
            auto midpoint = sdsl::middle(result[i]);
            
            // Extract 3D position (x, y, z)
            data_ptr[i * 3 + 0] = static_cast<float>(midpoint.getXDouble());
            data_ptr[i * 3 + 1] = static_cast<float>(midpoint.getYDouble());
            data_ptr[i * 3 + 2] = static_cast<float>(midpoint.getZDouble());
        }
        
        // Publish the pointcloud
        pcPublisher_->publish(pointcloud_msg);

    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDSL_ROS2>());
    rclcpp::shutdown();
    return 0;
}