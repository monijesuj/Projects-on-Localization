#include <Eigen/Dense>

#include "localization/localization_node.hpp"
#include "localization/scan.hpp"
#include "localization/tools.hpp"

LocalizationNode::LocalizationNode()
    : rclcpp::Node("localization_node")
{
    // Declare all parameters
    this->declare_parameter<std::string>("robot_name", "");

    this->declare_parameter<double>("worldX", 0.);
    this->declare_parameter<double>("worldY", 0.);
    this->declare_parameter<double>("worldBorder", 0.);

    this->declare_parameter<double>("beaconWidth", 0.);
    this->declare_parameter<double>("beaconBorder", 0.);
    this->declare_parameter<double>("beaconRadius", 0.);
    this->declare_parameter<double>("beaconRange", 0.);
    this->declare_parameter<int>("minPointsPerBeacon", 0);

    this->declare_parameter<double>("sigmaCoord", 0.);
    this->declare_parameter<double>("sigmaAngle", 0.);
    this->declare_parameter<double>("sigmaR", 0.);
    this->declare_parameter<double>("sigmaPhi", 0.);
    this->declare_parameter<double>("maxDist", 0.);
    this->declare_parameter<double>("minSin", 0.);
    this->declare_parameter<int>("particleCount", 0);

    // UKF specific parameters
    this->declare_parameter<double>("ukf_alpha", 0.001);
    this->declare_parameter<double>("ukf_beta", 2.0);
    this->declare_parameter<double>("ukf_kappa", 0.0);
    this->declare_parameter<bool>("use_ukf", true);

    this->declare_parameter<std::vector<double>>("startPose", std::vector<double> { 0., 0., 0. });
    this->declare_parameter<std::vector<double>>("lidarOffset", std::vector<double> { 0., 0., 0. });
    this->declare_parameter<int>("plateNumber", 0);
    this->declare_parameter<bool>("sixBeacons", 0);

    this->declare_parameter<double>("minEnemyRadius", 0.);
    this->declare_parameter<double>("maxEnemyDetectionSize", 0.);
    this->declare_parameter<double>("enemyMergeRadius", 0.);
    this->declare_parameter<double>("cvMergeRadius", 0.);
    this->declare_parameter<double>("enemyDataTimeout", 0.);
    this->declare_parameter<double>("relativeDistanceThreshold", 0.);

    this->declare_parameter<std::string>("startBypass", "");
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<bool>("publishAll", false);

    // Load parameters
    this->robot_name = this->get_parameter("robot_name").as_string();
    auto worldX = this->get_parameter("worldX").as_double();
    auto worldY = this->get_parameter("worldY").as_double();
    auto worldBorder = this->get_parameter("worldBorder").as_double();

    auto beaconWidth = this->get_parameter("beaconWidth").as_double();
    auto beaconBorder = this->get_parameter("beaconBorder").as_double();
    this->beaconRadius = this->get_parameter("beaconRadius").as_double();
    auto beaconRange = this->get_parameter("beaconRange").as_double();
    auto minPointsPerBeacon = this->get_parameter("minPointsPerBeacon").as_int();
    auto maxDist = this->get_parameter("maxDist").as_double();
    auto minSin = this->get_parameter("minSin").as_double();

    auto sigmaCoord = this->get_parameter("sigmaCoord").as_double();
    auto sigmaAngle = this->get_parameter("sigmaAngle").as_double();
    auto sigmaR = this->get_parameter("sigmaR").as_double();
    auto sigmaPhi = this->get_parameter("sigmaPhi").as_double();
    auto particleCount = this->get_parameter("particleCount").as_int();

    // UKF specific parameters
    auto ukf_alpha = this->get_parameter("ukf_alpha").as_double();
    auto ukf_beta = this->get_parameter("ukf_beta").as_double();
    auto ukf_kappa = this->get_parameter("ukf_kappa").as_double();
    this->useUKF = this->get_parameter("use_ukf").as_bool();

    this->startPose = Pose(this->get_parameter("startPose").as_double_array().data());
    this->lidarOffset = Pose(this->get_parameter("lidarOffset").as_double_array().data());
    int plateNumber = this->get_parameter("plateNumber").as_int();
    bool sixBeacons = this->get_parameter("sixBeacons").as_bool();

    auto minEnemyRadius = this->get_parameter("minEnemyRadius").as_double();
    auto maxEnemyDetectionSize = this->get_parameter("maxEnemyDetectionSize").as_double();
    auto enemyMergeRadius = this->get_parameter("enemyMergeRadius").as_double();
    auto cvMergeRadius = this->get_parameter("cvMergeRadius").as_double();
    auto enemyDataTimeout = this->get_parameter("enemyDataTimeout").as_double();
    auto relativeDistanceThreshold = this->get_parameter("relativeDistanceThreshold").as_double();

    auto startBypass = this->get_parameter("startBypass").as_string();
    auto debug = this->get_parameter("debug").as_bool();
    this->publishAll = this->get_parameter("publishAll").as_bool();

    // Set rcutils logging level
    if (debug)
    {
        auto res = rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        (void)res;
    }

    this->robotPose = this->startPose;
    RCLCPP_INFO(this->get_logger(), "Start pose: (%lf, %lf, %lf)", this->robotPose[0], this->robotPose[1], this->robotPose[2]);

   // Calculate world beacon positions using corrected offsets
    auto sideOffsetX = 0.05;
    auto sideOffsetY = 0.05;

    // Blue beacons: two on top, one on bottom
    this->blueBeacons.resize(2, 3);
    this->blueBeacons << sideOffsetX,            worldX - sideOffsetX, worldX / 2,
                        worldY + sideOffsetY,     worldY + sideOffsetY, -sideOffsetY;

    // Green beacons: one on top, two on bottom
    this->greenBeacons.resize(2, 3);
    this->greenBeacons << worldX / 2,            sideOffsetX,            worldX - sideOffsetX,
                        worldY + sideOffsetY,     -sideOffsetY,          -sideOffsetY;

    // Combine all beacons
    if (sixBeacons)
    {
        this->landmarks.resize(2, 6);
        this->landmarks << this->blueBeacons, this->greenBeacons;
    }
    else
    {
        this->landmarks.resize(2, 3);
        if (plateNumber == 0)
            this->landmarks = this->blueBeacons;
        else
            this->landmarks = this->greenBeacons;
    }

    // Create sensors
    Pose covNoise(sigmaCoord, sigmaCoord, sigmaAngle);
    Pose lidarCov(0.01, 0.01, 0.01);
    Pose cameraShift(0.01, 0.01, 0.01);
    PoseArray cameraScale(0.01, 0.01, 0.01);
    Pose wheelOdomRelativeNoiseStd(0.1, 0.1, 0.1);

    auto lidarSensor = std::make_shared<LiDAR>(0.5, Eigen::Array3i(1, 1, 1), lidarCov);
    auto cameraSensor = std::make_shared<Camera>(0.3, Eigen::Array3i(1, 1, 0), cameraShift, cameraScale);
    auto wheelOdomSensor = std::make_shared<WheelOdom>(0.2, Eigen::Array3i(1, 1, 1), wheelOdomRelativeNoiseStd);

    this->sensorMap["LiDAR"] = lidarSensor;
    this->sensorMap["Camera"] = cameraSensor;
    this->sensorMap["WheelOdom"] = wheelOdomSensor;

    std::vector<std::shared_ptr<Sensor>> sensors = { lidarSensor, cameraSensor, wheelOdomSensor };
    int fallbackSensorIndex = 0;

    // Initialize filter based on user preference
    if (this->useUKF) {
        // Initialize UKF
        PoseCov initialCovariance = covNoise.asDiagonal();
        this->unscentedKalmanFilter = std::make_unique<UnscentedKalmanFilter>(
            this->get_logger(),
            this->startPose,
            initialCovariance,
            ukf_alpha,
            ukf_beta,
            ukf_kappa,
            sensors,
            fallbackSensorIndex,
            this->landmarks);
        
        RCLCPP_INFO(this->get_logger(), "Using Unscented Kalman Filter for localization");
    } else {
        // Initialize Particle Filter
        this->particleFilter = std::make_unique<ParticleFilter>(
            this->get_logger(),
            covNoise,
            sigmaR,
            sigmaPhi,
            this->startPose,
            particleCount,
            sensors,
            fallbackSensorIndex,
            this->landmarks);
        
        RCLCPP_INFO(this->get_logger(), "Using Particle Filter for localization");
    }

    // Initialize object detector
    this->objectDetector = std::make_unique<ObjectDetector>(
        minEnemyRadius,
        maxEnemyDetectionSize,
        enemyMergeRadius,
        cvMergeRadius,
        enemyDataTimeout,
        relativeDistanceThreshold);

    // Initialize TF broadcaster
    this->baseFrameBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize publishers
    this->beaconPublisher = this->create_publisher<MarkerArrayMsg>(
        this->robot_name + "/beacons", 10);
    this->robotPosePublisher = this->create_publisher<PoseCovStampedMsg>(
        this->robot_name + "/pose", 10);
    this->enemiesPublisher = this->create_publisher<PoseArrayMsg>(
        this->robot_name + "/enemies", 10);
    this->globalEnemiesPublisher = this->create_publisher<PoseArrayMsg>(
        this->robot_name + "/global_enemies", 10);

    if (this->publishAll)
    {
        this->weightsPublisher = this->create_publisher<Float64MultiArrayMsg>(
            this->robot_name + "/weights", 10);
        this->particlesPublisher = this->create_publisher<PoseArrayMsg>(
            this->robot_name + "/particles", 10);
        this->filterScanPublisher = this->create_publisher<MarkerArrayMsg>(
            this->robot_name + "/filtered_scan", 10);
    }

    // Initialize subscribers
    this->lidarScanSubscription = this->create_subscription<LaserScanMsg>(
        this->robot_name + "/scan", 10,
        std::bind(&LocalizationNode::lidarScanCallback, this, std::placeholders::_1));
    this->lidarOdomSubscription = this->create_subscription<OdometryMsg>(
        this->robot_name + "/odom", 10,
        std::bind(&LocalizationNode::lidarOdomCallback, this, std::placeholders::_1));
    this->wheelOdomSubscription = this->create_subscription<TwistStampedMsg>(
        this->robot_name + "/wheel_odom", 10,
        std::bind(&LocalizationNode::wheelOdomCallback, this, std::placeholders::_1));
    this->cameraPoseSubscription = this->create_subscription<PoseStampedMsg>(
        this->robot_name + "/camera_pose", 10,
        std::bind(&LocalizationNode::cameraPoseCallback, this, std::placeholders::_1));

    this->cvEnemy1Subscription = this->create_subscription<PoseStampedMsg>(
        this->robot_name + "/cv_enemy_1", 10,
        std::bind(&LocalizationNode::cvEnemy1Callback, this, std::placeholders::_1));
    this->cvEnemy2Subscription = this->create_subscription<PoseStampedMsg>(
        this->robot_name + "/cv_enemy_2", 10,
        std::bind(&LocalizationNode::cvEnemy2Callback, this, std::placeholders::_1));
    this->companionEnemiesSubscription = this->create_subscription<PoseArrayMsg>(
        this->robot_name + "/companion_enemies", 10,
        std::bind(&LocalizationNode::companionEnemiesCallback, this, std::placeholders::_1));
    this->companionRobotPoseSubscription = this->create_subscription<PoseCovStampedMsg>(
        this->robot_name + "/companion_robot_pose", 10,
        std::bind(&LocalizationNode::companionRobotPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Localization node initialized");
}

void LocalizationNode::lidarOdomCallback(const OdometryMsg& msg)
{
    auto& orientation = msg.pose.pose.orientation;
    Quaternion q(orientation.w, orientation.x, orientation.y, orientation.z);
    double angle = quaternionToEuler(q)(2);

    SensorData data;
    data << RCUTILS_NS_TO_S((double)rclcpp::Time(msg.header.stamp).nanoseconds()),
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        angle;
    wrapAngle(data.row(3));
    this->sensorMap.at("LiDAR")->setData(data);
}

void LocalizationNode::wheelOdomCallback(const TwistStampedMsg& msg)
{
    SensorData data;
    data << RCUTILS_NS_TO_S((double)rclcpp::Time(msg.header.stamp).nanoseconds()),
        msg.twist.linear.x,
        msg.twist.linear.y,
        msg.twist.angular.z;

    wrapAngle(data.row(3));
    this->sensorMap.at("WheelOdom")->setData(data);
}

void LocalizationNode::cameraPoseCallback(const PoseStampedMsg& msg)
{
    SensorData data;
    data << RCUTILS_NS_TO_S((double)rclcpp::Time(msg.header.stamp).nanoseconds()),
        msg.pose.position.x,
        msg.pose.position.y,
        0;

    this->sensorMap.at("Camera")->setData(data);
}

void LocalizationNode::cvEnemy1Callback(const PoseStampedMsg& msg)
{
    double recvTime = RCUTILS_NS_TO_S((double)rclcpp::Time(msg.header.stamp).nanoseconds());
    EnemyData data(recvTime, Point(msg.pose.position.x, msg.pose.position.y));
    this->objectDetector->setEnemy1CVData(data);
}

void LocalizationNode::cvEnemy2Callback(const PoseStampedMsg& msg)
{
    double recvTime = RCUTILS_NS_TO_S((double)rclcpp::Time(msg.header.stamp).nanoseconds());
    EnemyData data(recvTime, Point(msg.pose.position.x, msg.pose.position.y));
    this->objectDetector->setEnemy2CVData(data);
}

void LocalizationNode::companionEnemiesCallback(const PoseArrayMsg& msg)
{
    Points enemies(2, msg.poses.size());
    for (size_t i = 0; i < msg.poses.size(); i++)
    {
        enemies(0, i) = msg.poses[i].position.x;
        enemies(1, i) = msg.poses[i].position.y;
    }

    this->objectDetector->setCompanionEnemies(enemies);
}

void LocalizationNode::companionRobotPoseCallback(const PoseCovStampedMsg& msg)
{
    Point companionRobotPosition = Point(msg.pose.pose.position.x, msg.pose.pose.position.y);
    this->objectDetector->setCompanionRobotPosition(companionRobotPosition);
}

void LocalizationNode::publishBaseFrame(const Pose& coords)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = this->robot_name + "_start";
    t.child_frame_id = this->robot_name + "_base";
    t.transform.translation.x = coords(0);
    t.transform.translation.y = coords(1);
    t.transform.translation.z = 0;

    Pose euler(0., 0., coords(2));
    Quaternion q = eulerToQuaternion(euler);
    t.transform.rotation.w = q.w();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();

    this->baseFrameBroadcaster->sendTransform(t);
}

void LocalizationNode::publishBeacons(const Points& beacons)
{
    MarkerArrayMsg markerArray;
    markerArray.markers.resize(beacons.cols());

    for (auto i = 0; i < beacons.cols(); i++)
    {
        auto& marker = markerArray.markers[i];
        marker.header.frame_id = this->robot_name + "_base";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "beacons";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = beacons(0, i);
        marker.pose.position.y = beacons(1, i);
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = this->beaconRadius * 2;
        marker.scale.y = this->beaconRadius * 2;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }

    this->beaconPublisher->publish(markerArray);
}

void LocalizationNode::publishRobotPose(const PoseCov& covariance)
{
    PoseCovStampedMsg msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = this->robot_name + "_start";
    msg.pose.pose.position.x = this->robotPose(0);
    msg.pose.pose.position.y = this->robotPose(1);
    msg.pose.pose.position.z = 0;

    Pose euler(0., 0., this->robotPose(2));
    Quaternion q = eulerToQuaternion(euler);
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();

    for (auto i = 0; i < 3; i++)
    {
        for (auto j = 0; j < 3; j++)
        {
            msg.pose.covariance[i * 6 + j] = covariance(i, j);
        }
    }

    this->robotPosePublisher->publish(msg);
}

void LocalizationNode::publishEnemies()
{
    auto enemies = this->objectDetector->getEnemies();
    PoseArrayMsg msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = this->robot_name + "_base";
    msg.poses.resize(enemies.cols());

    for (auto i = 0; i < enemies.cols(); i++)
    {
        msg.poses[i].position.x = enemies(0, i);
        msg.poses[i].position.y = enemies(1, i);
        msg.poses[i].position.z = 0;
        msg.poses[i].orientation.w = 1;
    }

    this->enemiesPublisher->publish(msg);
}

void LocalizationNode::publishGlobalEnemies()
{
    auto enemies = this->objectDetector->getGlobalEnemies();
    PoseArrayMsg msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = this->robot_name + "_start";
    msg.poses.resize(enemies.cols());

    for (auto i = 0; i < enemies.cols(); i++)
    {
        msg.poses[i].position.x = enemies(0, i);
        msg.poses[i].position.y = enemies(1, i);
        msg.poses[i].position.z = 0;
        msg.poses[i].orientation.w = 1;
    }

    this->globalEnemiesPublisher->publish(msg);
}

void LocalizationNode::publishWeights()
{
    if (!this->publishAll || this->useUKF)
        return;

    Float64MultiArrayMsg msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "weights";
    msg.layout.dim[0].size = this->particleFilter->getWeights().size();
    msg.layout.dim[0].stride = 1;
    msg.data.resize(this->particleFilter->getWeights().size());

    for (auto i = 0; i < this->particleFilter->getWeights().size(); i++)
    {
        msg.data[i] = this->particleFilter->getWeights()(i);
    }

    this->weightsPublisher->publish(msg);
}

void LocalizationNode::publishParticles()
{
    if (!this->publishAll || this->useUKF)
        return;

    PoseArrayMsg msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = this->robot_name + "_start";
    msg.poses.resize(this->particleFilter->getParticles().cols());

    for (auto i = 0; i < this->particleFilter->getParticles().cols(); i++)
    {
        msg.poses[i].position.x = this->particleFilter->getParticles()(0, i);
        msg.poses[i].position.y = this->particleFilter->getParticles()(1, i);
        msg.poses[i].position.z = 0;

        Pose euler(0., 0., this->particleFilter->getParticles()(2, i));
        Quaternion q = eulerToQuaternion(euler);
        msg.poses[i].orientation.w = q.w();
        msg.poses[i].orientation.x = q.x();
        msg.poses[i].orientation.y = q.y();
        msg.poses[i].orientation.z = q.z();
    }

    this->particlesPublisher->publish(msg);
}

void LocalizationNode::publishFilteredScan()
{
    if (!this->publishAll)
        return;

    auto filteredScan = this->objectDetector->getFilteredScan();
    MarkerArrayMsg markerArray;
    markerArray.markers.resize(filteredScan.cols());

    for (auto i = 0; i < filteredScan.cols(); i++)
    {
        auto& marker = markerArray.markers[i];
        marker.header.frame_id = this->robot_name + "_base";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "filtered_scan";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = filteredScan(0, i);
        marker.pose.position.y = filteredScan(1, i);
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }

    this->filterScanPublisher->publish(markerArray);
}

void LocalizationNode::lidarScanCallback(const LaserScanMsg& msg)
{
    // Process LiDAR scan
    Scan scan(msg);
    auto maxDist = this->get_parameter("maxDist").as_double();
    auto minSin = this->get_parameter("minSin").as_double();
    scan.filter(0.05, maxDist, minSin);

    // Detect beacons
    auto beaconRange = this->get_parameter("beaconRange").as_double();
    auto minPointsPerBeacon = this->get_parameter("minPointsPerBeacon").as_int();
    auto beacons = this->objectDetector->detectBeacons(scan.getPoints(), scan.getIntensities(), beaconRange, minPointsPerBeacon);

    // Detect enemies
    this->objectDetector->detectEnemies(scan.getPoints(), this->robotPose);

    // Publish beacons
    this->publishBeacons(beacons);

    // Publish enemies
    this->publishEnemies();
    this->publishGlobalEnemies();

    // Publish filtered scan
    this->publishFilteredScan();

    // Get current time
    double currentTime = RCUTILS_NS_TO_S((double)rclcpp::Time(msg.header.stamp).nanoseconds());

    // Run localization
    Gaussian filteredPose;
    if (this->useUKF) {
        filteredPose = this->unscentedKalmanFilter->localize(currentTime, beacons);
    } else {
        filteredPose = this->particleFilter->localize(currentTime, beacons);
    }

    // Update robot pose
    this->robotPose = filteredPose.mu;

    // Publish robot pose
    this->publishRobotPose(filteredPose.Sigma);

    // Publish base frame
    this->publishBaseFrame(this->robotPose);

    // Publish weights and particles (for particle filter)
    this->publishWeights();
    this->publishParticles();
}
