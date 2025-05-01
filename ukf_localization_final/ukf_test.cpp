#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "localization/ukf.hpp"
#include "localization/tools.hpp"

// Simple test function to validate UKF implementation
void test_ukf() {
    // Create a logger
    auto logger = rclcpp::get_logger("ukf_test");
    
    // Initial pose and covariance
    Pose initialPose(0.5, 0.5, 0.0);
    PoseCov initialCovariance = Pose(0.1, 0.1, 0.1).asDiagonal();
    
    // UKF parameters
    double alpha = 0.001;
    double beta = 2.0;
    double kappa = 0.0;
    
    // Create landmarks (beacons)
    Points landmarks(2, 3);
    landmarks << 0.0, 2.0, 1.0,
                 0.0, 0.0, 2.0;
    
    // Create sensors
    Pose lidarCov(0.01, 0.01, 0.01);
    Pose wheelOdomRelativeNoiseStd(0.1, 0.1, 0.1);
    
    auto lidarSensor = std::make_shared<LiDAR>(0.5, Eigen::Array3i(1, 1, 1), lidarCov);
    auto wheelOdomSensor = std::make_shared<WheelOdom>(0.5, Eigen::Array3i(1, 1, 1), wheelOdomRelativeNoiseStd);
    
    std::vector<std::shared_ptr<Sensor>> sensors = { lidarSensor, wheelOdomSensor };
    int fallbackSensorIndex = 0;
    
    // Create UKF
    UnscentedKalmanFilter ukf(
        logger,
        initialPose,
        initialCovariance,
        alpha,
        beta,
        kappa,
        sensors,
        fallbackSensorIndex,
        landmarks
    );
    
    // Simulate wheel odometry data
    SensorData odomData;
    odomData << 0.0, 0.1, 0.0, 0.0;  // time, vx, vy, omega
    wheelOdomSensor->setData(odomData);
    
    // Simulate LiDAR beacon detection
    Points beacons(2, 1);
    beacons << 1.0, 0.5;  // One beacon at (1.0, 0.5)
    
    // Run localization
    double currentTime = 0.1;
    Gaussian filteredPose = ukf.localize(currentTime, beacons);
    
    // Print results
    std::cout << "Filtered pose: " << filteredPose.mu.transpose() << std::endl;
    std::cout << "Covariance: " << std::endl << filteredPose.Sigma << std::endl;
    
    // Verify results
    bool testPassed = true;
    
    // Check if pose is reasonable (should have moved in x direction)
    if (filteredPose.mu(0) <= initialPose(0)) {
        std::cout << "Test failed: X position should have increased" << std::endl;
        testPassed = false;
    }
    
    // Check if covariance is positive definite
    Eigen::LLT<PoseCov> lltOfCov(filteredPose.Sigma);
    if (lltOfCov.info() != Eigen::Success) {
        std::cout << "Test failed: Covariance is not positive definite" << std::endl;
        testPassed = false;
    }
    
    if (testPassed) {
        std::cout << "UKF test passed!" << std::endl;
    } else {
        std::cout << "UKF test failed!" << std::endl;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    test_ukf();
    rclcpp::shutdown();
    return 0;
}
