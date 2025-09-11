#include <vector>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class LocalizationPublisher : public rclcpp::Node {
public:
    LocalizationPublisher() : Node("localization_publisher"), hasValidTransform_(false) {
        sdslWithScoresSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "sdsl_with_scores", 10, 
            std::bind(&LocalizationPublisher::sdslWithScoresCallback, this, std::placeholders::_1));
        
        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
        
        transformTimer_ = this->create_wall_timer(
            100ms,  // 10Hz = 100ms interval
            std::bind(&LocalizationPublisher::publishLastTransform, this));
        
        RCLCPP_INFO(this->get_logger(), "LocalizationPublisher node started with 10Hz transform publisher");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sdslWithScoresSubscriber_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    
    rclcpp::Time lastPublishTime_;
    const std::chrono::duration<double> publishInterval_{1.0}; // 1 second
    
    rclcpp::TimerBase::SharedPtr transformTimer_;
    geometry_msgs::msg::TransformStamped lastMapToOdom_;
    bool hasValidTransform_;

    void publishLastTransform() {
        if (!hasValidTransform_) {
            return;
        }
        
        lastMapToOdom_.header.stamp = this->now();
        tfBroadcaster_->sendTransform(lastMapToOdom_);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Re-published last map->odom transform at 10Hz: (%.3f, %.3f, %.3f)", 
            lastMapToOdom_.transform.translation.x,
            lastMapToOdom_.transform.translation.y, 
            lastMapToOdom_.transform.translation.z);
    }

    void sdslWithScoresCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", msg->width);
        
        if (msg->width == 0) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }

        // Extract all points and scores
        std::vector<std::tuple<float, float, float, float>> points; // x, y, z, score
        for (size_t i = 0; i < msg->width; ++i) {
            float x = *reinterpret_cast<const float*>(&msg->data[i * msg->point_step + msg->fields[0].offset]);
            float y = *reinterpret_cast<const float*>(&msg->data[i * msg->point_step + msg->fields[1].offset]);
            float z = *reinterpret_cast<const float*>(&msg->data[i * msg->point_step + msg->fields[2].offset]);
            float score = *reinterpret_cast<const float*>(&msg->data[i * msg->point_step + msg->fields[3].offset]);
            points.emplace_back(x, y, z, score);
        }

        // Sort by score (highest first)
        std::sort(points.begin(), points.end(), 
                  [](const auto& a, const auto& b) { return std::get<3>(a) > std::get<3>(b); });

        float maxScore = std::get<3>(points[0]);
        float maxX = std::get<0>(points[0]);
        float maxY = std::get<1>(points[0]);
        float maxZ = std::get<2>(points[0]);

        RCLCPP_INFO(this->get_logger(), 
            "Publishing best localization: (%.3f, %.3f, %.3f) with score %.6f", 
            maxX, maxY, maxZ, maxScore);

        // Get the current odom->base_footprint transform (use latest available)
        geometry_msgs::msg::TransformStamped odomToBase;
        try {
            odomToBase = tfBuffer_->lookupTransform(
                "odom", "base_footprint", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), 
                "Could not get odom->base_footprint transform: %s", ex.what());
            return;
        }

        // Compute map->odom transform
        // We want: map->base_footprint = map->odom * odom->base_footprint
        // So: map->odom = map->base_footprint * inv(odom->base_footprint)
        
        // map->base_footprint is the localization result (maxX, maxY, 0 with no rotation)
        geometry_msgs::msg::TransformStamped mapToBase;
        mapToBase.header.stamp = msg->header.stamp;
        mapToBase.header.frame_id = "map";
        mapToBase.child_frame_id = "base_footprint";
        mapToBase.transform.translation.x = maxX;
        mapToBase.transform.translation.y = maxY;
        mapToBase.transform.translation.z = 0.0;
        mapToBase.transform.rotation.x = 0.0;
        mapToBase.transform.rotation.y = 0.0;
        mapToBase.transform.rotation.z = 0.0;
        mapToBase.transform.rotation.w = 1.0;

        // Compute map->odom = map->base_footprint * inv(odom->base_footprint)
        geometry_msgs::msg::TransformStamped mapToOdom;
        mapToOdom.header.stamp = msg->header.stamp;
        mapToOdom.header.frame_id = "map";
        mapToOdom.child_frame_id = "odom";

        // Convert to tf2 transforms for computation
        tf2::Transform tf_mapToBase;
        tf2::fromMsg(mapToBase.transform, tf_mapToBase);
        
        tf2::Transform tf_odomToBase;
        tf2::fromMsg(odomToBase.transform, tf_odomToBase);
        
        // Compute map->odom = map->base * inv(odom->base)
        tf2::Transform tf_mapToOdom = tf_mapToBase * tf_odomToBase.inverse();
        
        // Convert back to message
        tf2::toMsg(tf_mapToOdom, mapToOdom.transform);

        // Store the transform for periodic publishing
        lastMapToOdom_ = mapToOdom;
        hasValidTransform_ = true;

        // Publish the map->odom transform with current timestamp for broadcasting
        mapToOdom.header.stamp = this->now();
        tfBroadcaster_->sendTransform(mapToOdom);

        RCLCPP_DEBUG(this->get_logger(), 
            "Published map->odom transform: (%.3f, %.3f, %.3f)", 
            mapToOdom.transform.translation.x,
            mapToOdom.transform.translation.y, 
            mapToOdom.transform.translation.z);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationPublisher>());
    rclcpp::shutdown();
    return 0;
}