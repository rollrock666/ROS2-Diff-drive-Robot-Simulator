#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Eigen>
#include <memory>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

// 可调参数结构体
struct RobotSimulatorParams {
    // 模型参数
    std::string mesh_resource = "package://simulation_env/meshes/robot.dae";
    double mesh_scale = 0.0005;
    double mesh_offset_x = -0.15;  // 模型中心偏移
    double mesh_offset_y = -0.16;

    // 控制参数
    double max_linear_vel = 1.0;   // 最大线速度
    double max_angular_vel = M_PI; // 最大角速度
    double control_rate = 100.0;   // 控制频率 (Hz)

    // 点云参数
    std::string pcd_file_path = "/home/tsm/simulation_ws/src/simulation_env/PCD/forest.pcd";
    double search_radius = 0.5;    // 邻近点搜索半径
    int min_neighbors = 10;        // 最小邻近点数

    // 高度估计参数
    double height_offset = 0.0;    // 高度偏移
};

class RobotSimulator : public rclcpp::Node {
public:
    RobotSimulator() : Node("robot_simulator"), tf_broadcaster_(this), params_() {
        // 初始化发布器和订阅器
        model_pub_ = create_publisher<visualization_msgs::msg::Marker>("/robot_model", 10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 1);
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                linear_vel_ = std::clamp(msg->linear.x, -params_.max_linear_vel, params_.max_linear_vel);
                angular_vel_ = std::clamp(msg->angular.z, -params_.max_angular_vel, params_.max_angular_vel);
            });

        // 初始化定时器
        double dt = 1.0 / params_.control_rate;
        timer_ = create_wall_timer(std::chrono::duration<double>(dt), [this]() {
            updateRobotPose();
            estimateHeightAndPose();
            publishModel();
            broadcastTransform();
            publishOdometry();
            publishPointCloud();
        });

        // 初始化位姿和点云
        robot_pose_ = {3.0, 0.0, 0.0, 0.0};
        loadPointCloud();
    }

private:
    // 可调参数
    RobotSimulatorParams params_;

    // 点云相关
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_;
    double current_height_ = 0.0;
    tf2::Quaternion current_orientation_;

    // 机器人状态
    struct Pose {
        double x, y, z, theta;
    } robot_pose_;
    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;

    // ROS 接口
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr model_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 加载点云
    void loadPointCloud() {
        world_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(params_.pcd_file_path, *world_cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file");
            return;
        }

        // 计算点云质心并平移
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*world_cloud_, centroid);
        for (auto& point : world_cloud_->points) {
            point.x -= centroid[0];
            point.y -= centroid[1];
            point.z -= centroid[2];
        }

        // 设置点云坐标系
        world_cloud_->header.frame_id = "map";
        kd_tree_.setInputCloud(world_cloud_);
        RCLCPP_INFO(this->get_logger(), "Loaded %d points", world_cloud_->size());
    }

    // 发布点云
    void publishPointCloud() {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*world_cloud_, cloud_msg);
        cloud_msg.header.stamp = now();
        cloud_msg.header.frame_id = "map";
        pointcloud_pub_->publish(cloud_msg);
    }

    // 估计高度和姿态
    void estimateHeightAndPose() {
        pcl::PointXYZ search_point{robot_pose_.x, robot_pose_.y, 0};
        std::vector<int> indices;
        std::vector<float> distances;

        if (kd_tree_.radiusSearch(search_point, params_.search_radius, indices, distances) > params_.min_neighbors) {
            // PCA 法线估计
            Eigen::Matrix3f covariance;
            Eigen::Vector4f centroid;
            pcl::computeMeanAndCovarianceMatrix(*world_cloud_, indices, covariance, centroid);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
            Eigen::Vector3f normal = solver.eigenvectors().col(0);
            normal.normalize();

            if (normal.z() < 0) normal = -normal;
            current_height_ = centroid.z();

            // 构建旋转矩阵
            Eigen::Vector3f z_axis = normal;
            Eigen::Vector3f heading_dir(cos(robot_pose_.theta), sin(robot_pose_.theta), 0);
            Eigen::Vector3f x_axis = heading_dir.cross(z_axis).normalized();
            Eigen::Vector3f y_axis = z_axis.cross(x_axis);

            Eigen::Matrix3f rot_matrix;
            rot_matrix.col(0) = x_axis;
            rot_matrix.col(1) = y_axis;
            rot_matrix.col(2) = z_axis;

            // 转换为四元数
            Eigen::Quaternionf q(rot_matrix);
            current_orientation_.setValue(q.x(), q.y(), q.z(), q.w());
            tf2::Quaternion q_offset;
            q_offset.setRPY(0, 0, M_PI / 2);  // 绕 Z 轴旋转 90°
            current_orientation_ = current_orientation_ * q_offset;
        }
    }

    // 更新机器人位姿
    void updateRobotPose() {
        double dt = 1.0 / params_.control_rate;
        if (std::abs(angular_vel_) > 1e-5) {
            double radius = linear_vel_ / angular_vel_;
            robot_pose_.x += radius * (sin(robot_pose_.theta + angular_vel_ * dt) - sin(robot_pose_.theta));
            robot_pose_.y += radius * (cos(robot_pose_.theta) - cos(robot_pose_.theta + angular_vel_ * dt));
        } else {
            robot_pose_.x += linear_vel_ * dt * cos(robot_pose_.theta);
            robot_pose_.y += linear_vel_ * dt * sin(robot_pose_.theta);
        }
        robot_pose_.theta = fmod(robot_pose_.theta + angular_vel_ * dt, 2 * M_PI);
    }

    // 发布模型
    void publishModel() {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "robot";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.mesh_resource = params_.mesh_resource;
        marker.mesh_use_embedded_materials = true;

        double dx = params_.mesh_offset_x;
        double dy = params_.mesh_offset_y;
        marker.pose.position.x = robot_pose_.x + dx * cos(robot_pose_.theta) - dy * sin(robot_pose_.theta);
        marker.pose.position.y = robot_pose_.y + dx * sin(robot_pose_.theta) + dy * cos(robot_pose_.theta);
        marker.pose.position.z = current_height_ + params_.height_offset;

        tf2::Quaternion q;
        q.setRPY(M_PI / 2, 0, M_PI / 2);
        marker.pose.orientation = tf2::toMsg(current_orientation_ * q);

        marker.scale.x = marker.scale.y = marker.scale.z = params_.mesh_scale;
        model_pub_->publish(marker);
    }

    // 发布 TF
    void broadcastTransform() {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = robot_pose_.x;
        transform.transform.translation.y = robot_pose_.y;
        transform.transform.translation.z = current_height_ + params_.height_offset;
        transform.transform.rotation = tf2::toMsg(current_orientation_);

        tf_broadcaster_.sendTransform(transform);
    }

    // 发布里程计
    void publishOdometry() {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = robot_pose_.x;
        odom_msg.pose.pose.position.y = robot_pose_.y;
        odom_msg.pose.pose.position.z = current_height_ + params_.height_offset;
        odom_msg.pose.pose.orientation = tf2::toMsg(current_orientation_);

        odom_msg.twist.twist.linear.x = linear_vel_;
        odom_msg.twist.twist.angular.z = angular_vel_;

        odom_pub_->publish(odom_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotSimulator>());
    rclcpp::shutdown();
    return 0;
}