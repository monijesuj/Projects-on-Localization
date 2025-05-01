#ifndef UKF_HPP
#define UKF_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include "localization/sensor.hpp"
#include "localization/types.hpp"

// Gaussian distribution structure (same as in particle_filter.hpp for compatibility)
struct Gaussian
{
    Gaussian(Pose mu, PoseCov Sigma)
        : mu(mu)
        , Sigma(Sigma)
    {
    }

    Pose mu;
    PoseCov Sigma;
};

class UnscentedKalmanFilter
{
public:
    UnscentedKalmanFilter(
        const rclcpp::Logger& logger,
        const Pose& initialPose,
        const PoseCov& initialCovariance,
        double alpha,
        double beta,
        double kappa,
        std::vector<std::shared_ptr<Sensor>>& sensors,
        int fallbackSensorIndex,
        const Points& landmarks);

    Gaussian localize(double referenceTime, const Points& beacons);

private:
    // UKF core functions
    void generateSigmaPoints();
    void predictState(double dt);
    void updateWithOdometry(const SensorData& odomData);
    void updateWithLidar(const Points& beacons);
    
    // Helper functions
    bool distributeSensorWeights(Array& weights);
    void updateMotionModel();
    void updateMeasurementModel(const Points& beacons);
    Gaussian getGaussianStatistics();
    
    // UKF parameters
    double alpha, beta, kappa, lambda;
    int stateDim, augmentedDim;
    
    // State and covariance
    Vector stateVector;
    Matrix stateCovariance;
    
    // Sigma points and weights
    Matrix sigmaPoints;
    Vector meanWeights, covWeights;
    
    // Noise covariances
    Matrix processNoise;
    Matrix odomMeasurementNoise;
    Matrix lidarMeasurementNoise;
    
    // Other members
    rclcpp::Logger logger;
    std::vector<std::shared_ptr<Sensor>> sensors;
    int fallbackSensorIndex;
    Points landmarks;
    Pose prevPose;
    Pose startPose;
};

#endif // UKF_HPP
