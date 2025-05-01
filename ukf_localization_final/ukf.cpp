#include "localization/ukf.hpp"
#include "localization/tools.hpp"
#include <iostream>

UnscentedKalmanFilter::UnscentedKalmanFilter(
    const rclcpp::Logger& logger,
    const Pose& initialPose,
    const PoseCov& initialCovariance,
    double alpha,
    double beta,
    double kappa,
    std::vector<std::shared_ptr<Sensor>>& sensors,
    int fallbackSensorIndex,
    const Points& landmarks)
    : logger(logger)
    , alpha(alpha)
    , beta(beta)
    , kappa(kappa)
    , sensors(sensors)
    , fallbackSensorIndex(fallbackSensorIndex)
    , landmarks(landmarks)
    , startPose(initialPose)
{
    // Initialize state dimensions
    stateDim = 6;  // [x, y, theta, vx, vy, omega]
    augmentedDim = stateDim * 2;  // State + process noise
    
    // Calculate lambda parameter
    lambda = alpha * alpha * (stateDim + kappa) - stateDim;
    
    // Initialize state vector
    stateVector = Vector::Zero(stateDim);
    stateVector.head(3) = initialPose;  // Set position and orientation
    
    // Initialize covariance matrix
    stateCovariance = Matrix::Zero(stateDim, stateDim);
    stateCovariance.topLeftCorner(3, 3) = initialCovariance;
    stateCovariance.bottomRightCorner(3, 3) = Matrix::Identity(3, 3) * 0.1;  // Initial velocity uncertainty
    
    // Initialize sigma points matrix
    sigmaPoints = Matrix::Zero(stateDim, 2 * stateDim + 1);
    
    // Initialize weights
    meanWeights = Vector::Zero(2 * stateDim + 1);
    covWeights = Vector::Zero(2 * stateDim + 1);
    
    // Calculate weights
    meanWeights(0) = lambda / (stateDim + lambda);
    covWeights(0) = meanWeights(0) + (1 - alpha * alpha + beta);
    
    for (int i = 1; i < 2 * stateDim + 1; i++) {
        meanWeights(i) = 1.0 / (2 * (stateDim + lambda));
        covWeights(i) = meanWeights(i);
    }
    
    // Initialize noise covariances
    processNoise = Matrix::Identity(stateDim, stateDim);
    processNoise.topLeftCorner(3, 3) *= 0.01;  // Position and orientation noise
    processNoise.bottomRightCorner(3, 3) *= 0.1;  // Velocity noise
    
    odomMeasurementNoise = Matrix::Identity(3, 3) * 0.01;  // Odometry measurement noise
    lidarMeasurementNoise = Matrix::Identity(2, 2);  // LiDAR measurement noise
    lidarMeasurementNoise(0, 0) = 0.01;  // Range noise
    lidarMeasurementNoise(1, 1) = 0.01;  // Bearing noise
    
    this->prevPose = this->startPose;
    
    RCLCPP_INFO(this->logger, "UKF initialized, state dimension: %d", stateDim);
}

void UnscentedKalmanFilter::generateSigmaPoints() {
    // Calculate square root of scaled covariance matrix
    Eigen::LLT<Matrix> lltOfCov(stateCovariance);
    Matrix L = lltOfCov.matrixL();  // Lower triangular matrix
    Matrix sqrtScaledCov = std::sqrt(stateDim + lambda) * L;
    
    // Set first sigma point as the mean
    sigmaPoints.col(0) = stateVector;
    
    // Generate remaining sigma points
    for (int i = 0; i < stateDim; i++) {
        sigmaPoints.col(i + 1) = stateVector + sqrtScaledCov.col(i);
        sigmaPoints.col(i + 1 + stateDim) = stateVector - sqrtScaledCov.col(i);
    }
    
    // Wrap angle for all sigma points
    for (int i = 0; i < 2 * stateDim + 1; i++) {
        wrapAngle(sigmaPoints.col(i).row(2));
    }
}

void UnscentedKalmanFilter::predictState(double dt) {
    // Generate sigma points
    generateSigmaPoints();
    
    // Predict sigma points
    Matrix predictedSigmaPoints = Matrix::Zero(stateDim, 2 * stateDim + 1);
    
    for (int i = 0; i < 2 * stateDim + 1; i++) {
        // Extract state components
        double x = sigmaPoints(0, i);
        double y = sigmaPoints(1, i);
        double theta = sigmaPoints(2, i);
        double vx = sigmaPoints(3, i);
        double vy = sigmaPoints(4, i);
        double omega = sigmaPoints(5, i);
        
        // Predict using motion model
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);
        
        // Motion model for omnidirectional robot
        predictedSigmaPoints(0, i) = x + (vx * cos_theta - vy * sin_theta) * dt;
        predictedSigmaPoints(1, i) = y + (vx * sin_theta + vy * cos_theta) * dt;
        predictedSigmaPoints(2, i) = theta + omega * dt;
        predictedSigmaPoints(3, i) = vx;  // Assume constant velocity
        predictedSigmaPoints(4, i) = vy;  // Assume constant velocity
        predictedSigmaPoints(5, i) = omega;  // Assume constant angular velocity
        
        // Wrap angle
        wrapAngle(predictedSigmaPoints.col(i).row(2));
    }
    
    // Calculate predicted mean
    stateVector.setZero();
    for (int i = 0; i < 2 * stateDim + 1; i++) {
        stateVector += meanWeights(i) * predictedSigmaPoints.col(i);
    }
    
    // Wrap angle in state vector
    wrapAngle(stateVector.row(2));
    
    // Calculate predicted covariance
    stateCovariance.setZero();
    for (int i = 0; i < 2 * stateDim + 1; i++) {
        Vector diff = predictedSigmaPoints.col(i) - stateVector;
        // Wrap angle difference
        wrapAngle(diff.row(2));
        
        stateCovariance += covWeights(i) * diff * diff.transpose();
    }
    
    // Add process noise
    stateCovariance += processNoise;
    
    // Update sigma points with predicted state and covariance
    sigmaPoints = predictedSigmaPoints;
}

bool UnscentedKalmanFilter::distributeSensorWeights(Array& weights) {
    // This function is similar to the one in ParticleFilter
    int enabledSensorCnt = 0;
    double enabledTotalWeight = 0.;

    auto totalSensorCnt = this->sensors.size();
    weights.fill(0.);

    // Calculate total weight of enabled sensors
    for (size_t i = 0; i < totalSensorCnt; i++) {
        auto& sensor = this->sensors.at(i);
        if (sensor->isEnabled()) {
            weights(i) = sensor->getWeight();
            enabledTotalWeight += weights(i);
            enabledSensorCnt++;
        }
    }

    // Check if any sensors enabled
    if (enabledSensorCnt == 0) {
        RCLCPP_WARN(this->logger, "All sensors disabled");
        return false;
    }

    std::string enabledSensorsStr = "";
    for (const auto& sensor : this->sensors) {
        if (sensor->isEnabled())
            enabledSensorsStr += sensor->getName() + ", ";
    }
    enabledSensorsStr = std::to_string(enabledSensorCnt) + " sensors enabled: " + enabledSensorsStr;
    RCLCPP_DEBUG(this->logger, "%s", enabledSensorsStr.c_str());

    weights += (1 - enabledTotalWeight) / enabledSensorCnt;
    return true;
}

void UnscentedKalmanFilter::updateMotionModel() {
    // Update weights for enabled sensors
    Array sensorWeights(this->sensors.size());
    bool sensorsEnabled = this->distributeSensorWeights(sensorWeights);

    if (not sensorsEnabled)
        return;

    // Determine fallback sensor
    int currentFallbackIndex = this->fallbackSensorIndex;
    if (not this->sensors.at(currentFallbackIndex)->isEnabled()) // Fallback sensor is disabled
    {
        currentFallbackIndex = -1;

        for (size_t i = 0; i < this->sensors.size(); i++) {
            auto& sensor = this->sensors.at(i);
            if (sensor->isEnabled() and sensor->areAllChannelsEnabled()) {
                currentFallbackIndex = i;
                RCLCPP_DEBUG(this->logger, "Switched fallback sensor to: %d", currentFallbackIndex);
                break;
            }
        }
    }

    // Process each sensor's data
    for (size_t i = 0; i < this->sensors.size(); i++) {
        auto& sensor = this->sensors.at(i);
        if (not sensor->isEnabled())
            continue;

        // Get displacement from sensor
        Pose displacement = sensor->getPoseDelta();
        
        // Update state based on sensor type
        if (sensor->getName() == "WheelOdom") {
            // Update velocities in state vector
            stateVector(3) = displacement(0);  // vx
            stateVector(4) = displacement(1);  // vy
            stateVector(5) = displacement(2);  // omega
            
            // Update process noise based on sensor covariance
            PoseCov sensorCov = sensor->getCovariance();
            processNoise.bottomRightCorner(3, 3) = sensorCov;
        }
        else if (sensor->getName() == "LiDAR") {
            // For LiDAR, we'll use the measurement update instead
            // This is just a placeholder for any direct state updates needed
        }
    }
    
    // Predict state using motion model
    double dt = 0.01;  // Default time step, should be calculated from sensor timestamps
    predictState(dt);
}

void UnscentedKalmanFilter::updateWithLidar(const Points& beacons) {
    int beaconCount = beacons.cols();
    int landmarkCount = this->landmarks.cols();

    // If no beacons found, skip update
    if (beaconCount == 0) {
        RCLCPP_DEBUG(this->logger, "No beacons found, skipping LiDAR update");
        return;
    }

    // For each beacon, find the closest landmark and update
    for (int beac_i = 0; beac_i < beaconCount; beac_i++) {
        Point beacon = beacons.col(beac_i);
        
        // Find closest landmark to this beacon
        int closestLandmarkIdx = -1;
        double minDist = std::numeric_limits<double>::max();
        
        for (int lm_i = 0; lm_i < landmarkCount; lm_i++) {
            Point landmark = this->landmarks.col(lm_i);
            double dist = (landmark - beacon).norm();
            
            if (dist < minDist) {
                minDist = dist;
                closestLandmarkIdx = lm_i;
            }
        }
        
        if (closestLandmarkIdx == -1) {
            continue;  // No landmark found
        }
        
        Point closestLandmark = this->landmarks.col(closestLandmarkIdx);
        
        // Generate sigma points
        generateSigmaPoints();
        
        // Transform sigma points through measurement model
        Matrix measSigmaPoints = Matrix::Zero(2, 2 * stateDim + 1);
        
        for (int i = 0; i < 2 * stateDim + 1; i++) {
            // Extract state components
            double x = sigmaPoints(0, i);
            double y = sigmaPoints(1, i);
            double theta = sigmaPoints(2, i);
            
            // Calculate expected measurement (range and bearing)
            double dx = closestLandmark(0) - x;
            double dy = closestLandmark(1) - y;
            
            double range = std::sqrt(dx*dx + dy*dy);
            double bearing = std::atan2(dy, dx) - theta;
            
            // Wrap bearing angle
            while (bearing > M_PI) bearing -= 2.0 * M_PI;
            while (bearing < -M_PI) bearing += 2.0 * M_PI;
            
            measSigmaPoints(0, i) = range;
            measSigmaPoints(1, i) = bearing;
        }
        
        // Calculate predicted measurement mean
        Vector measPred = Vector::Zero(2);
        for (int i = 0; i < 2 * stateDim + 1; i++) {
            measPred += meanWeights(i) * measSigmaPoints.col(i);
        }
        
        // Calculate predicted measurement covariance
        Matrix measCov = Matrix::Zero(2, 2);
        for (int i = 0; i < 2 * stateDim + 1; i++) {
            Vector diff = measSigmaPoints.col(i) - measPred;
            // Wrap bearing difference
            while (diff(1) > M_PI) diff(1) -= 2.0 * M_PI;
            while (diff(1) < -M_PI) diff(1) += 2.0 * M_PI;
            
            measCov += covWeights(i) * diff * diff.transpose();
        }
        
        // Add measurement noise
        measCov += lidarMeasurementNoise;
        
        // Calculate cross-correlation matrix
        Matrix crossCorr = Matrix::Zero(stateDim, 2);
        for (int i = 0; i < 2 * stateDim + 1; i++) {
            Vector stateDiff = sigmaPoints.col(i) - stateVector;
            // Wrap angle difference
            wrapAngle(stateDiff.row(2));
            
            Vector measDiff = measSigmaPoints.col(i) - measPred;
            // Wrap bearing difference
            while (measDiff(1) > M_PI) measDiff(1) -= 2.0 * M_PI;
            while (measDiff(1) < -M_PI) measDiff(1) += 2.0 * M_PI;
            
            crossCorr += covWeights(i) * stateDiff * measDiff.transpose();
        }
        
        // Calculate Kalman gain
        Matrix kalmanGain = crossCorr * measCov.inverse();
        
        // Calculate actual measurement
        double dx = closestLandmark(0) - stateVector(0);
        double dy = closestLandmark(1) - stateVector(1);
        
        double actualRange = std::sqrt(dx*dx + dy*dy);
        double actualBearing = std::atan2(dy, dx) - stateVector(2);
        
        // Wrap bearing angle
        while (actualBearing > M_PI) actualBearing -= 2.0 * M_PI;
        while (actualBearing < -M_PI) actualBearing += 2.0 * M_PI;
        
        Vector actualMeas(2);
        actualMeas << actualRange, actualBearing;
        
        // Calculate measurement residual
        Vector residual = beacon - actualMeas;
        // Wrap bearing residual
        while (residual(1) > M_PI) residual(1) -= 2.0 * M_PI;
        while (residual(1) < -M_PI) residual(1) += 2.0 * M_PI;
        
        // Update state
        stateVector += kalmanGain * residual;
        // Wrap angle in state
        wrapAngle(stateVector.row(2));
        
        // Update covariance
        stateCovariance -= kalmanGain * measCov * kalmanGain.transpose();
        
        // Ensure covariance remains symmetric and positive definite
        stateCovariance = (stateCovariance + stateCovariance.transpose()) / 2.0;
    }
}

void UnscentedKalmanFilter::updateMeasurementModel(const Points& beacons) {
    // Update with LiDAR measurements
    updateWithLidar(beacons);
}

Gaussian UnscentedKalmanFilter::getGaussianStatistics() {
    // Extract pose and covariance
    Pose mu = stateVector.head(3);
    PoseCov Sigma = stateCovariance.topLeftCorner(3, 3);
    
    return Gaussian(mu, Sigma);
}

Gaussian UnscentedKalmanFilter::localize(double referenceTime, const Points& beacons) {
    // Validate data (enables or disables sensors)
    for (auto& sensor : this->sensors)
        sensor->validateData(referenceTime);

    // Run UKF
    this->updateMotionModel();
    this->updateMeasurementModel(beacons);

    Gaussian filteredPose = this->getGaussianStatistics();

    // Indicate sensor data as used
    for (auto& sensor : this->sensors)
        sensor->setUsed();

    this->prevPose = filteredPose.mu;

    return filteredPose;
}
