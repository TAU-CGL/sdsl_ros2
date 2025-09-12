#include <vector>
#include <map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sdsl_ros2/msg/point_cloud_with_transform.hpp"

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
            if (points[i].x() == nearest.x() && points[i].y() == nearest.y() && points[i].z() == nearest.z()) return i;
        }
        return -1;
    }
};

class SDSL_OdometryFusion : public rclcpp::Node {
public:
    SDSL_OdometryFusion() :
        Node("odometry_fusion") {
        sdslRawSubscriber_ = this->create_subscription<sdsl_ros2::msg::PointCloudWithTransform>(
            "sdsl_raw", 10, std::bind(&SDSL_OdometryFusion::sdslRawCallback, this, std::placeholders::_1));
        sdslWithScoresPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdsl_with_scores", 10);
    }

private:
    // Subscriptions and publishes
    rclcpp::Subscription<sdsl_ros2::msg::PointCloudWithTransform>::SharedPtr sdslRawSubscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sdslWithScoresPublisher_;
    
    double gamma_ = 0.999; // Forgetting rate
    double epsilon_ = 0.1; // Error standard deviation
    PointCloud prevPointCloud_;

    static double logaddexp(double a, double b) {
        if (a == -std::numeric_limits<double>::infinity()) return b;
        if (b == -std::numeric_limits<double>::infinity()) return a;
        return std::max(a, b) + std::log1p(std::exp(-std::abs(a - b)));
    }

    void sdslRawCallback(const sdsl_ros2::msg::PointCloudWithTransform::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received SDSL raw point cloud with %d points", msg->point_cloud.width);

        PointCloud pointCloud;
        pointCloud.points = pointCloudFromMsg(std::make_shared<sensor_msgs::msg::PointCloud2>(msg->point_cloud));
        pointCloud.timestamp = msg->point_cloud.header.stamp;
        pointCloud.transform = msg->transform;
        pointCloud.isNull = false;
        pointCloud.buildKdTree();

        // We now have a good point cloud

        if (prevPointCloud_.isNull || prevPointCloud_.points.empty()) {
            // First point cloud, initialize scores to 1.0
            // double initialScore = -std::log(pointCloud.points.size());
            double initialScore = 1.0 / pointCloud.points.size();
            pointCloud.scores = std::vector<double>(pointCloud.points.size(), initialScore);
            publishPointCloudWithScores(pointCloud);
            prevPointCloud_ = pointCloud;
            RCLCPP_INFO(this->get_logger(), "Initialized first point cloud with uniform scores");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Fusing with previous point cloud with %d points", prevPointCloud_.points.size());

        double totalScore = 0.0;
        // double totalScore = -std::numeric_limits<double>::infinity();
        Eigen::Matrix3d T = computeTransformDifference(
            prevPointCloud_.transform, pointCloud.transform);

        // (Using log-scores) \int P[X_t|X_{t-1}] * Bel(X_{t-1}) dX_{t-1} 
        for (size_t i = 0; i < pointCloud.points.size(); ++i) {
            Kernel::Point_3 pt = pointCloud.points[i];
            Kernel::Point_3 Uinv_pt = transformPoint(T, pt);

            double score = 0.0;
            // double score = -std::numeric_limits<double>::infinity();
            for (size_t j = 0; j < prevPointCloud_.points.size(); ++j) {
                double dist = CGAL::squared_distance(Uinv_pt, prevPointCloud_.points[j]);
                double pxt_xtminusone = std::exp(-dist / (2 * epsilon_ * epsilon_)) / (std::sqrt(2 * M_PI) * epsilon_);
                score += prevPointCloud_.scores[j] * pxt_xtminusone;
                // double log_pxt_xtminusone = -dist / (2 * epsilon_ * epsilon_) - 1.5 * std::log(2 * M_PI * epsilon_ * epsilon_);
                // double log_prior = prevPointCloud_.scores[j] + std::log(gamma_);
                // score = logaddexp(score, log_prior + log_pxt_xtminusone);
            }

            totalScore += score;
            // totalScore = logaddexp(totalScore, score);
            pointCloud.scores.push_back(score);
        }
        // Normalize scores
        for (double& score : pointCloud.scores) {
            score /= totalScore;
            // score -= totalScore;
        }

        publishPointCloudWithScores(pointCloud);
        prevPointCloud_ = pointCloud;
    }

    void publishPointCloudWithScores(const PointCloud& pointCloud) {
        sensor_msgs::msg::PointCloud2 pointcloud_msg;
        pointcloud_msg.header.stamp = pointCloud.timestamp;
        pointcloud_msg.header.frame_id = "map";
        pointcloud_msg.height = 1;
        pointcloud_msg.width = pointCloud.points.size();
        pointcloud_msg.is_dense = true;
        
        // Define point cloud fields (x, y, z, score)
        sensor_msgs::msg::PointField field_x, field_y, field_z, field_score;
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

        field_score.name = "score";
        field_score.offset = 12;
        field_score.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_score.count = 1;
                
        pointcloud_msg.fields = {field_x, field_y, field_z, field_score};
        pointcloud_msg.point_step = 4 * 4; // 4 floats * 4 bytes each
        pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width;
        
        // Populate point cloud data
        pointcloud_msg.data.resize(pointcloud_msg.row_step);
        float* data_ptr = reinterpret_cast<float*>(pointcloud_msg.data.data());
        
        for (size_t i = 0; i < pointCloud.points.size(); ++i) {
            const Kernel::Point_3& pt = pointCloud.points[i];
            double score = pointCloud.scores[i];
            
            data_ptr[i * 4 + 0] = static_cast<float>(pt.x());
            data_ptr[i * 4 + 1] = static_cast<float>(pt.y());
            data_ptr[i * 4 + 2] = static_cast<float>(pt.z());
            data_ptr[i * 4 + 3] = static_cast<float>(score);
        }
 
        sdslWithScoresPublisher_->publish(pointcloud_msg);
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