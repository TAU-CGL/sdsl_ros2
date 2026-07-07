#include <map>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cassert>

#define SDSL_CPP_ONLY
#include <sdsl/sdsl.hpp>
#include <sdsl/environments/env_pgm.hpp>
#include <sdsl/predicates/pred_forward_2d.hpp>
#include <CGAL/Simple_cartesian.h>
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

using namespace std::chrono_literals;

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using LaserScan = sensor_msgs::msg::LaserScan;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using PointCloud = sensor_msgs::msg::PointCloud2;

class SDSL_ROS2 : public rclcpp::Node {
public:
    SDSL_ROS2() : Node("sdsl_ros2"), environmentInitialized_(false) {
        using std::placeholders::_1;
        mapSubscription_ = this->create_subscription<OccupancyGrid>(
            "map", 10, std::bind(&SDSL_ROS2::mapCallback, this, _1)); 
        sdsSubscription_ = this->create_subscription<LaserScan>(
            "sds", 10, std::bind(&SDSL_ROS2::sdsCallback, this, _1));
        pcPublisher_ = this->create_publisher<PointCloud>("sdsl", 10);

        // 'Subscribe' to the pose of the robot
        tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        // Declare parameters
        kk_prime_ratio_ = this->declare_parameter<double>("kk_prime_ratio", 0.8);
        error_bound_ = this->declare_parameter<double>("error_bound", 0.005);
        recursion_depth_ = this->declare_parameter<int>("recursion_depth", 7);
        timeout_ = this->declare_parameter<int>("timeout", 1000.0);
        
        RCLCPP_INFO(this->get_logger(), "SDSL_ROS2 node has been started.");
	}

private:
    rclcpp::Subscription<OccupancyGrid>::SharedPtr mapSubscription_;
    rclcpp::Subscription<LaserScan>::SharedPtr sdsSubscription_;
    rclcpp::Publisher<PointCloud>::SharedPtr pcPublisher_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    TransformStamped transform_, previousTransform_;

    std::shared_ptr<sdsl::Env_PGM<3>> environment_;
    bool environmentInitialized_;

    double kk_prime_ratio_;
    double error_bound_;
    int recursion_depth_;
    double timeout_;


    // --------------------------------------------------------------------------
    void mapCallback(const OccupancyGrid::SharedPtr msg) {
        printMapStats(msg);
        loadMapFromOccupancyGridMessage(msg);
        RCLCPP_INFO(this->get_logger(), "Environment constructed.");
    }

    void sdsCallback(const LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received laser scan with %zu ranges", msg->ranges.size());
        if (!environmentInitialized_) return;
        try {
            captureRobotTransform();
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        } // Do not run localization until we can capture the robot's actual odometry

        std::vector<sdsl::Configuration<3>> odometries;
        std::vector<FT> measurements;
        getOdometriesAndMeasurements(msg, odometries, measurements);

        //std::vector<sdsl::Voxel<3>> localization = 
        getLocalization(odometries, measurements);
        //std::vector<double> belief = getLocalizationBeliefScores(localization);

        //publishLocalizationPointCloud(localization, belief);
    }

    void loadMapFromOccupancyGridMessage(const OccupancyGrid::SharedPtr msg) {
        int width = msg->info.width;
        int height = msg->info.height;
        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;
        std::vector<uint8_t> pixels = convertOccupancyGridToPGM(msg);
        environment_ = std::make_shared<sdsl::Env_PGM<3>>();
        environment_->load(pixels.data(), width, height, resolution, origin_x, origin_y);
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

    void captureRobotTransform() {
        TransformStamped tmpTransform = tfBuffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero); // May raise exception
        previousTransform_ = transform_;
        transform_ = tmpTransform;
    }

     void getOdometriesAndMeasurements(const LaserScan::SharedPtr msg,std::vector<sdsl::Configuration<3>>& odometries,std::vector<FT>& measurements) {
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max || std::isnan(range)) {
                continue; // Ignore invalid ranges
            }
            double angle = msg->angle_min + i * msg->angle_increment;
            sdsl::Configuration<3> g(0.0, 0.0, angle);
            odometries.push_back(g);
            measurements.push_back(range);
        }
    }

    std::vector<sdsl::Voxel<3>> getLocalization(std::vector<sdsl::Configuration<3>>& odometries, std::vector<FT>& measurements) {
        sdsl::Predicate_Fwd2D<3,FT> predicate(environment_, odometries, measurements, kk_prime_ratio_, error_bound_);
        auto start = std::chrono::steady_clock::now();
        auto result = localize_omp_forkjoin(environment_->boundingBox(), predicate, recursion_depth_, timeout_, true);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        RCLCPP_INFO(this->get_logger(), "Localization result: %d", result.size());
        RCLCPP_INFO(this->get_logger(), "Localization took: %.3f seconds", elapsed.count());
    }

    std::vector<FT> getLocalizationBeliefScores(std::vector<sdsl::Voxel<3>> localization) {
        std::vector<FT> belief;
        return belief;
    }

    void publishLocalizationPointCloud(std::vector<sdsl::Voxel<3>> localization, std::vector<FT> belief) {
        sensor_msgs::msg::PointCloud2 msg;
        pointcloud_msg.header.stamp = this->now();
        pointcloud_msg.header.frame_id = "map";
        pointcloud_msg.height = 1; // Unordered point cloud
        pointcloud_msg.width = localization.size();
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
        for(size_t i = 0; i < localization.size(); ++i) {
            sdsl::Configuration<3> midpoint = localization[i].midpoint();
            data_ptr[i * 3 + 0] = static_cast<float>(midpoint[0]);
            data_ptr[i * 3 + 1] = static_cast<float>(midpoint[1]);
            data_ptr[i * 3 + 2] = static_cast<float>(midpoint[2]);
        }
        pcPublisher_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDSL_ROS2>());
    rclcpp::shutdown();
    return 0;
}
