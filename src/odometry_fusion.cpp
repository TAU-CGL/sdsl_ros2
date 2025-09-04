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

struct TransformWithTimestamp {
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Time timestamp;
};

struct PointCloudWithTimestampAndScores {
    std::vector<Kernel::Point_3> points;
    std::vector<double> scores; // One score per point
    rclcpp::Time timestamp;
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
        timer_ = this->create_wall_timer(100ms, std::bind(&SDSL_OdometryFusion::transformCallback, this));
    }

private:
    // Subscriptions and publishes
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sdslRawSubscriber_;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    rclcpp::TimerBase::SharedPtr timer_;
    

    double lambda_ = 0.99; // Forgetting rate
    std::vector<TransformWithTimestamp> transformHistory_;
    PointCloudWithTimestampAndScores prevPointCloud_;

    void sdslRawCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received SDSL raw point cloud with %d points", msg->width);

        // First, check if there is a transform older then 

        PointCloudWithTimestampAndScores pointCloud;
        pointCloud.points = pointCloudFromMsg(msg);
        pointCloud.timestamp = msg->header.stamp;
        pointCloud.isNull = false;

        if (prevPointCloud_.isNull) {
            // First point cloud, initialize scores to 1.0
            pointCloud.scores = std::vector<double>(pointCloud.points.size(), 1.0 / pointCloud.points.size());
            prevPointCloud_ = pointCloud;
            RCLCPP_INFO(this->get_logger(), "Initialized first point cloud with uniform scores");
            return;
        }
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

            transformHistory_.push_back({transform, this->now()});
        } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    void getInterpolatedTransform(double queryTime) {
        if (transformHistory_.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Not enough transform history for interpolation");
            return;
        }

        // Find two transforms surrounding the query time
        TransformWithTimestamp before, after;
        for (size_t i = 1; i < transformHistory_.size(); ++i) {
            if (transformHistory_[i].timestamp.seconds() >= queryTime) {
                before = transformHistory_[i-1];
                after = transformHistory_[i];
                break;
            }
        }

        double t = (queryTime - before.timestamp.seconds()) /
                (after.timestamp.seconds() - before.timestamp.seconds());

        // Linear interpolation of translation
        geometry_msgs::msg::Vector3 interpTranslation;
        interpTranslation.x = (1 - t) * before.transform.transform.translation.x +
                              t * after.transform.transform.translation.x;
        interpTranslation.y = (1 - t) * before.transform.transform.translation.y +
                              t * after.transform.transform.translation.y;
        interpTranslation.z = (1 - t) * before.transform.transform.translation.z +
                              t * after.transform.transform.translation.z;

        // Slerp for rotation
        tf2::Quaternion q_before, q_after, q_interp;
        tf2::fromMsg(before.transform.transform.rotation, q_before);
        tf2::fromMsg(after.transform.transform.rotation, q_after);
        q_interp = q_before.slerp(q_after, t);

        geometry_msgs::msg::Quaternion interpRotation = tf2::toMsg(q_interp);

        RCLCPP_INFO(this->get_logger(), 
            "Interpolated Transform at time %.3f:", queryTime);
        RCLCPP_INFO(this->get_logger(), 
            "  Translation: [x=%.3f, y=%.3f, z=%.3f]",
            interpTranslation.x,
            interpTranslation.y,
            interpTranslation.z);
        RCLCPP_INFO(this->get_logger(), 
            "  Rotation: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
            interpRotation.x,
            interpRotation.y,
            interpRotation.z,
            interpRotation.w);
    }


    void deleteOldTransforms(double cutoffTime) {
        while (!transformHistory_.empty() && 
            transformHistory_.front().timestamp.seconds() < cutoffTime) {
            transformHistory_.erase(transformHistory_.begin());
        }
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDSL_OdometryFusion>());
    rclcpp::shutdown();
    return 0;
}