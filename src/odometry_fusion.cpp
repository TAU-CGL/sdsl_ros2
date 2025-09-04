#include <vector>
#include <map>
#include <cmath>

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

#include <Eigen/Dense>
#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/K_neighbor_search.h>
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

    // Nearest neighbor search
    using KdTree = CGAL::Kd_tree<CGAL::Search_traits_3<Kernel>>;
    std::shared_ptr<KdTree> kdTree;

    void buildKdTree() {
        if (points.empty()) return;
        kdTree = std::make_shared<KdTree>(points.begin(), points.end());
    }
    int nearestNeighborIndex(const Kernel::Point_3& query) {
        if (!kdTree) return -1;
        CGAL::K_neighbor_search<CGAL::Search_traits_3<Kernel>> knn(*kdTree, query, 1);
        auto it = knn.begin();
        if (it == knn.end()) return -1;
        const Kernel::Point_3& nearest = it->first;
        // Find index of nearest in original points
        for (size_t i = 0; i < points.size(); ++i) {
            if (points[i] == nearest) return i;
        }
        return -1;
    }
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
    double eps_ = 0.0001; // Small constant to avoid zero division
    PointCloud prevPointCloud_;

    void sdslRawCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received SDSL raw point cloud with %d points", msg->width);

        PointCloud pointCloud;
        pointCloud.points = pointCloudFromMsg(msg);
        pointCloud.timestamp = msg->header.stamp;
        pointCloud.isNull = false;
        pointCloud.buildKdTree();
        
        // Try getting the transform (if we can't, skip this message)
        try {
            pointCloud.transform = tfBuffer_->lookupTransform(
                "odom", "base_footprint", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        // We now have a good point cloud

        if (prevPointCloud_.isNull || prevPointCloud_.points.empty()) {
            // First point cloud, initialize scores to 1.0
            pointCloud.scores = std::vector<double>(pointCloud.points.size(), 1.0 / pointCloud.points.size());
            prevPointCloud_ = pointCloud;
            RCLCPP_INFO(this->get_logger(), "Initialized first point cloud with uniform scores");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Fusing with previous point cloud with %d points", prevPointCloud_.points.size());

        double totalScore = 0.0;
        Eigen::Matrix3d T = computeTransformDifference(
            prevPointCloud_.transform, pointCloud.transform);
        for (size_t i = 0; i < pointCloud.points.size(); ++i) {
            Kernel::Point_3 pt = pointCloud.points[i];
            pt = transformPoint(T, pt);
            int nnIndex = prevPointCloud_.nearestNeighborIndex(pt);
            double prevScore = (nnIndex >= 0) ? prevPointCloud_.scores[nnIndex] : 0.0;
            Kernel::Point_3 nnPt = (nnIndex >= 0) ? prevPointCloud_.points[nnIndex] : Kernel::Point_3(0,0,0);
            double dist = std::sqrt(CGAL::squared_distance(pt, nnPt));

            double newScore = 1.0 / (dist + eps_);
            totalScore += newScore;

            newScore *= pow(prevScore, lambda_);
            pointCloud.scores.push_back(newScore);
        }
        // Normalize scores
        for (double& score : pointCloud.scores) {
            score /= totalScore;
        }

        prevPointCloud_ = pointCloud;
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

    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    Eigen::Matrix3d createSE2Matrix(double x, double y, double yaw) {
        Eigen::Matrix3d T;
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        
        T << cos_yaw, -sin_yaw, x,
            sin_yaw,  cos_yaw, y,
            0,        0,       1;
        
        return T;
    }

    Eigen::Matrix3d inverseSE2Matrix(double x, double y, double yaw) {
        Eigen::Matrix3d T_inv;
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        
        T_inv << cos_yaw,  sin_yaw, -x * cos_yaw - y * sin_yaw,
                -sin_yaw,  cos_yaw,  x * sin_yaw - y * cos_yaw,
                0,        0,        1;
        
        return T_inv;
    }

    Eigen::Matrix3d computeTransformDifference(
        const geometry_msgs::msg::TransformStamped& T_prev,
        const geometry_msgs::msg::TransformStamped& T_curr) {
        
        // Extract SE(2) components from T_prev
        double x_prev = T_prev.transform.translation.x;
        double y_prev = T_prev.transform.translation.y;
        double yaw_prev = quaternionToYaw(T_prev.transform.rotation);
        
        // Extract SE(2) components from T_curr
        double x_curr = T_curr.transform.translation.x;
        double y_curr = T_curr.transform.translation.y;
        double yaw_curr = quaternionToYaw(T_curr.transform.rotation);
        
        // Create T_prev matrix
        Eigen::Matrix3d T_prev_mat = createSE2Matrix(x_prev, y_prev, yaw_prev);
        
        // Create inverse of T_curr matrix
        Eigen::Matrix3d T_curr_inv = inverseSE2Matrix(x_curr, y_curr, yaw_curr);
        
        // Compute T = T_prev * inv(T_curr)
        Eigen::Matrix3d T = T_prev_mat * T_curr_inv;
        
        return T;
    }

    Kernel::Point_3 transformPoint(const Eigen::Matrix3d& T, const Kernel::Point_3& point) {
        // Extract transformation components from SE(2) matrix
        double cos_theta = T(0, 0);
        double sin_theta = T(1, 0);
        double tx = T(0, 2);
        double ty = T(1, 2);
        
        // Apply transformation: [x', y'] = R * [x, y] + t
        double x_new = cos_theta * point.x() - sin_theta * point.y() + tx;
        double y_new = sin_theta * point.x() + cos_theta * point.y() + ty;
        
        return Kernel::Point_3(x_new, y_new, 0.0);
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDSL_OdometryFusion>());
    rclcpp::shutdown();
    return 0;
}