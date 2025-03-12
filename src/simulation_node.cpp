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

class RobotSimulator : public rclcpp::Node {
public:
    RobotSimulator() : Node("robot_simulator"), tf_broadcaster_(this) {
        // 从参数服务器加载参数
        declare_parameter<std::string>("mesh_resource", "package://simulation_env/meshes/robot.dae");
        declare_parameter<double>("mesh_scale", 0.0005);
        declare_parameter<double>("mesh_offset_x", -0.15);
        declare_parameter<double>("mesh_offset_y", -0.16);
        declare_parameter<double>("max_linear_vel", 1.0);
        declare_parameter<double>("max_angular_vel", M_PI);
        declare_parameter<double>("control_rate", 50.0);
        declare_parameter<std::string>("pcd_file_path", "/home/robot/simulation_ws/src/simulation_env/PCD/forest.pcd");
        declare_parameter<double>("search_radius", 0.5);
        declare_parameter<int>("min_neighbors", 20);
        declare_parameter<double>("height_offset", 0.0);
        declare_parameter<double>("init_x", 0.0);
        declare_parameter<double>("init_y", 0.0);
        declare_parameter<double>("init_z", 0.0);
        // 获取参数值
        mesh_resource_ = get_parameter("mesh_resource").as_string();
        mesh_scale_ = get_parameter("mesh_scale").as_double();
        mesh_offset_x_ = get_parameter("mesh_offset_x").as_double();
        mesh_offset_y_ = get_parameter("mesh_offset_y").as_double();
        max_linear_vel_ = get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = get_parameter("max_angular_vel").as_double();
        control_rate_ = get_parameter("control_rate").as_double();
        pcd_file_path_ = get_parameter("pcd_file_path").as_string();
        search_radius_ = get_parameter("search_radius").as_double();
        min_neighbors_ = get_parameter("min_neighbors").as_int();
        height_offset_ = get_parameter("height_offset").as_double();

        double init_x= get_parameter("init_x").as_double();
        double init_y= get_parameter("init_y").as_double();
        double init_z= get_parameter("init_z").as_double();
        // 初始化发布器和订阅器
        model_pub_ = create_publisher<visualization_msgs::msg::Marker>("/robot_model", 10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 1);
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                linear_vel_ = std::clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
                angular_vel_ = std::clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);
            });

        // 初始化定时器
        double dt = 1.0 / control_rate_;
        timer_ = create_wall_timer(std::chrono::duration<double>(dt), [this]() {
            updateRobotPose();
            estimateHeightAndPose();
            publishModel();
            broadcastTransform();
            publishOdometry();
            publishPointCloud();
        });

        // 初始化位姿和点云
        robot_pose_ = {init_x, init_y, init_z, 0.0};
        loadPointCloud();
    }

private:
    // 参数
    std::string mesh_resource_;
    double mesh_scale_;
    double mesh_offset_x_;
    double mesh_offset_y_;
    double max_linear_vel_;
    double max_angular_vel_;
    double control_rate_;
    std::string pcd_file_path_;
    double search_radius_;
    int min_neighbors_;
    double height_offset_;

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
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *world_cloud_) == -1) {
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

    void estimateHeightAndPose() {
        pcl::PointXYZ search_point{robot_pose_.x, robot_pose_.y, 0};
        std::vector<int> indices;
        std::vector<float> distances;

        // 动态调整搜索半径
        double search_radius = search_radius_;
        int num_neighbors = 0;
        while (num_neighbors < min_neighbors_ && search_radius <= 2.0 * search_radius_) {
            num_neighbors = kd_tree_.radiusSearch(search_point, search_radius, indices, distances);
            if (num_neighbors < min_neighbors_) {
                search_radius += 0.1;  // 逐步增大搜索半径
            }
        }

        if (num_neighbors > min_neighbors_) {
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
        } else {
            // 点云数据不足，使用估计值
            RCLCPP_WARN(this->get_logger(), "Not enough points for height estimation even after increasing search radius! Using last valid estimate.");

            // 估计高度
            double dt = 1.0 / control_rate_;
            double height_change = linear_vel_ * dt * sin(current_orientation_.getAngle());  // 假设高度变化与线速度和姿态有关
            current_height_ += height_change;

            // 估计姿态
            double angle_change = angular_vel_ * dt;
            tf2::Quaternion delta_q;
            delta_q.setRPY(0, 0, angle_change);
            current_orientation_ = current_orientation_ * delta_q;
            current_orientation_.normalize();
        }
    }

    // 更新机器人位姿
    void updateRobotPose() {
        double dt = 1.0 / control_rate_;
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

        marker.mesh_resource = mesh_resource_;
        marker.mesh_use_embedded_materials = true;

        double dx = mesh_offset_x_;
        double dy = mesh_offset_y_;
        marker.pose.position.x = robot_pose_.x + dx * cos(robot_pose_.theta) - dy * sin(robot_pose_.theta);
        marker.pose.position.y = robot_pose_.y + dx * sin(robot_pose_.theta) + dy * cos(robot_pose_.theta);
        marker.pose.position.z = current_height_ + height_offset_;

        tf2::Quaternion q;
        q.setRPY(M_PI / 2, 0, M_PI / 2);
        marker.pose.orientation = tf2::toMsg(current_orientation_ * q);

        marker.scale.x = marker.scale.y = marker.scale.z = mesh_scale_;
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
        transform.transform.translation.z = current_height_ + height_offset_;
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
        odom_msg.pose.pose.position.z = current_height_ + height_offset_;
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