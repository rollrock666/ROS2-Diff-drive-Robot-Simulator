#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>
#include <random>
#include <memory>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>

class SimulatedLidar : public rclcpp::Node {
public:
    SimulatedLidar() : Node("simulated_lidar"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/simulated_lidar", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SimulatedLidar::odomCallback, this, std::placeholders::_1));
        global_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/map_cloud", 10, std::bind(&SimulatedLidar::globalCloudCallback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimulatedLidar::timerCallback, this));
        
        this->declare_parameter<double>("horizontal_resolution", 0.005);
        this->declare_parameter<int>("num_laser_lines", 16);
        horizontal_resolution_ = this->get_parameter("horizontal_resolution").as_double();
        num_laser_lines_ = this->get_parameter("num_laser_lines").as_int();
        
        int horizontal_bins = static_cast<int>(2 * M_PI / horizontal_resolution_);
        bin_matrix_ = std::vector<std::vector<double>>(horizontal_bins, std::vector<double>(num_laser_lines_, std::numeric_limits<double>::max()));
    }

private:
    double MAX_RANGE = 5.0;
    double MIN_RANGE = 0.1;
    double horizontal_resolution_;
    int num_laser_lines_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    std::vector<std::vector<double>> bin_matrix_;
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = msg;
    }

    void globalCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        global_cloud_ = cloud;
    }

    void timerCallback() {
        if (!global_cloud_ || !current_odom_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for global cloud and odometry data...");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform("base_link", "map", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        Eigen::Affine3d transform_matrix = tf2::transformToEigen(transform);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*global_cloud_, *transformed_cloud, transform_matrix);

        for (auto& row : bin_matrix_) {
            std::fill(row.begin(), row.end(), std::numeric_limits<double>::max());
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> noise(0.0, 0.02);

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(transformed_cloud);

        for (const auto& point : transformed_cloud->points) {
            double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (range > MAX_RANGE || range < MIN_RANGE) continue;

            double horizontal_angle = std::atan2(point.y, point.x);
            double vertical_angle = std::atan2(point.z, std::hypot(point.x, point.y));

            int horizontal_bin = static_cast<int>((horizontal_angle + M_PI) / horizontal_resolution_);
            int vertical_bin = static_cast<int>((vertical_angle + M_PI / 2) / (M_PI / (num_laser_lines_ - 1)));
            
            horizontal_bin = std::clamp(horizontal_bin, 0, static_cast<int>(bin_matrix_.size() - 1));
            vertical_bin = std::clamp(vertical_bin, 0, static_cast<int>(bin_matrix_[0].size() - 1));
            
            if (range < bin_matrix_[horizontal_bin][vertical_bin]) {
                bin_matrix_[horizontal_bin][vertical_bin] = range + noise(gen);
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr simulated_lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t h = 0; h < bin_matrix_.size(); ++h) {
            for (size_t v = 0; v < bin_matrix_[0].size(); ++v) {
                if (bin_matrix_[h][v] < std::numeric_limits<double>::max()) {
                    double horizontal_angle = h * horizontal_resolution_ - M_PI;
                    double vertical_angle = v * (M_PI / (num_laser_lines_ - 1)) - M_PI / 2;
                    double range = bin_matrix_[h][v];

                    double x = range * std::cos(vertical_angle) * std::cos(horizontal_angle);
                    double y = range * std::cos(vertical_angle) * std::sin(horizontal_angle);
                    double z = range * std::sin(vertical_angle);

                    simulated_lidar_cloud->points.emplace_back(x, y, z);
                }
            }
        }

        sensor_msgs::msg::PointCloud2 lidar_msg;
        pcl::toROSMsg(*simulated_lidar_cloud, lidar_msg);
        lidar_msg.header.stamp = this->now();
        lidar_msg.header.frame_id = "base_link";
        lidar_pub_->publish(lidar_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatedLidar>());
    rclcpp::shutdown();
    return 0;
}