#ifndef LOCALIZATION_NODE_HPP
#define LOCALIZATION_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "localization/object_detector.hpp"
#include "localization/particle_filter.hpp"
#include "localization/ukf.hpp"  // Added UKF header
#include "localization/types.hpp"

typedef visualization_msgs::msg::MarkerArray MarkerArrayMsg;
typedef geometry_msgs::msg::PoseWithCovarianceStamped PoseCovStampedMsg;
typedef std_msgs::msg::String StringMsg;
typedef sensor_msgs::msg::LaserScan LaserScanMsg;
typedef nav_msgs::msg::Odometry OdometryMsg;
typedef geometry_msgs::msg::TwistStamped TwistStampedMsg;
typedef std_msgs::msg::Float64MultiArray Float64MultiArrayMsg;
typedef geometry_msgs::msg::PoseArray PoseArrayMsg;
typedef geometry_msgs::msg::PoseStamped PoseStampedMsg;

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode();

private:
    void lidarOdomCallback(const OdometryMsg& msg);
    void wheelOdomCallback(const TwistStampedMsg& msg);
    void cameraPoseCallback(const PoseStampedMsg& msg);
    void lidarScanCallback(const LaserScanMsg& msg);

    void cvEnemy1Callback(const PoseStampedMsg& msg);
    void cvEnemy2Callback(const PoseStampedMsg& msg);
    void companionEnemiesCallback(const PoseArrayMsg& msg);
    void companionRobotPoseCallback(const PoseCovStampedMsg& msg);

    void publishBaseFrame(const Pose& coords);
    void publishBeacons(const Points& beacons);
    void publishRobotPose(const PoseCov& covariance);
    void publishEnemies();
    void publishGlobalEnemies();

    void publishWeights();
    void publishParticles();
    void publishFilteredScan();

    std::string robot_name;
    Pose startPose;
    Pose robotPose;

    double beaconRadius;
    Points landmarks;
    Points blueBeacons;
    Points greenBeacons;
    Pose lidarOffset;

    bool publishAll;
    bool useUKF;  // Added flag to choose between UKF and particle filter

    std::unordered_map<std::string, std::shared_ptr<Sensor>> sensorMap;

    std::unique_ptr<ParticleFilter> particleFilter;
    std::unique_ptr<UnscentedKalmanFilter> unscentedKalmanFilter;  // Added UKF instance
    std::unique_ptr<ObjectDetector> objectDetector;
    std::unique_ptr<tf2_ros::TransformBroadcaster> baseFrameBroadcaster;

    rclcpp::Publisher<MarkerArrayMsg>::SharedPtr beaconPublisher;
    rclcpp::Publisher<PoseCovStampedMsg>::SharedPtr robotPosePublisher;

    rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr weightsPublisher;
    rclcpp::Publisher<PoseArrayMsg>::SharedPtr particlesPublisher;
    rclcpp::Publisher<MarkerArrayMsg>::SharedPtr filterScanPublisher;
    rclcpp::Publisher<PoseArrayMsg>::SharedPtr enemiesPublisher;
    rclcpp::Publisher<PoseArrayMsg>::SharedPtr globalEnemiesPublisher;

    rclcpp::Subscription<StringMsg>::SharedPtr commandSubscription;
    rclcpp::Subscription<LaserScanMsg>::SharedPtr lidarScanSubscription;
    rclcpp::Subscription<OdometryMsg>::SharedPtr lidarOdomSubscription;
    rclcpp::Subscription<TwistStampedMsg>::SharedPtr wheelOdomSubscription;
    rclcpp::Subscription<PoseStampedMsg>::SharedPtr cameraPoseSubscription;

    rclcpp::Subscription<PoseStampedMsg>::SharedPtr cvEnemy1Subscription;
    rclcpp::Subscription<PoseStampedMsg>::SharedPtr cvEnemy2Subscription;
    rclcpp::Subscription<PoseArrayMsg>::SharedPtr companionEnemiesSubscription;
    rclcpp::Subscription<PoseCovStampedMsg>::SharedPtr companionRobotPoseSubscription;
};

#endif // LOCALIZATION_NODE_HPP
