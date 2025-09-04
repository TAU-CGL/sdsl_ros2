#include <vector>
#include <map>

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

#include <CGAL/Simple_cartesian.h>
using Kernel = CGAL::Simple_cartesian<double>;
using FT = Kernel::FT;

using namespace std::chrono_literals;

struct PointCloud {
    std::vector<Kernel::Point_3> points;
    std::vector<double> scores; // One score per point
    rclcpp::Time timestamp;
    geometry_msgs::msg::TransformStamped transform; // Transform at the time of the point cloud
    bool isNull = true;
};

class SDSL_OdometryFusion : public rclcpp::Node {
public:
    SDSL_OdometryFusion() :
        Node("odometry_fusion") {
        sdslRawSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "sdsl_raw", 10, std::bind(&SDSL_OdometryFusion::sdslRawCallback, this, std::placeholders::_1));

        tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    }

private:
    // Subscriptions and publishes
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sdslRawSubscriber_;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    
    double lambda_ = 0.99; // Forgetting rate
    PointCloud prevPointCloud_;

    void sdslRawCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received SDSL raw point cloud with %d points", msg->width);

        PointCloud pointCloud;
        pointCloud.points = pointCloudFromMsg(msg);
        pointCloud.timestamp = msg->header.stamp;
        pointCloud.isNull = false;

        // Try getting the transform (if we can't, skip this message)
        try {
            pointCloud.transform = tfBuffer_->lookupTransform(
                "odom", "base_footprint", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        // We now have a good point cloud

        if (prevPointCloud_.isNull) {
            // First point cloud, initialize scores to 1.0
            pointCloud.scores = std::vector<double>(pointCloud.points.size(), 1.0 / pointCloud.points.size());
            prevPointCloud_ = pointCloud;
            RCLCPP_INFO(this->get_logger(), "Initialized first point cloud with uniform scores");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Fusing with previous point cloud with %d points", prevPointCloud_.points.size());
    }

    std::vector<Kernel::Point_3> pointCloudFromMsg(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::vector<Kernel::Point_3> points;
        // Assuming msg is organized as x,y,z float32
        for (size_t i = 0; i < msg->width; ++i) {
            float wx = *reinterpret_cast<const float*>(&msg->data[i * msg->point_step + msg->fields[0].offset]);
            float wy = *reinterpret_cast<const float*>(&msg->data[i * msg->point_step + msg->fields[1].offset]);
            float wz = *reinterpret_cast<const float*>(&msg->data[i * msg->point_step + msg->fields[2].offset]);
            Kernel::Point_3 pt(wx, wy, wz);
            points.push_back(pt);
        }
        return points;
    }

    void transformCallback() {
        try {
            // Get transform from base_footprint to odom
            geometry_msgs::msg::TransformStamped transform = tfBuffer_->lookupTransform(
                "odom", "base_footprint", tf2::TimePointZero);
            // Print the transform details
            // RCLCPP_INFO(this->get_logger(), 
            //     "Transform base_footprint -> odom:");
            // RCLCPP_INFO(this->get_logger(), 
            //     "  Translation: [x=%.3f, y=%.3f, z=%.3f]",
            //     transform.transform.translation.x,
            //     transform.transform.translation.y,
            //     transform.transform.translation.z);
            // RCLCPP_INFO(this->get_logger(), 
            //     "  Rotation: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
            //     transform.transform.rotation.x,
            //     transform.transform.rotation.y,
            //     transform.transform.rotation.z,
            //     transform.transform.rotation.w);
        } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }


};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDSL_OdometryFusion>());
    rclcpp::shutdown();
    return 0;
}