# UKF Parameter Tuning Guide

This document provides guidance on tuning the Unscented Kalman Filter (UKF) parameters for optimal performance in the Eurobot competition environment.

## Core UKF Parameters

### Alpha (α)
- **Purpose**: Controls the spread of sigma points around the mean state
- **Default value**: 0.001
- **Tuning range**: 0.0001 to 0.1
- **Effect of increasing**: Wider spread of sigma points, more emphasis on capturing nonlinearities
- **Effect of decreasing**: Tighter spread of sigma points, more emphasis on the mean estimate
- **When to increase**: If the filter is not capturing highly nonlinear behaviors during rapid maneuvers
- **When to decrease**: If the filter is too sensitive to noise or producing erratic estimates

### Beta (β)
- **Purpose**: Incorporates prior knowledge about the distribution
- **Default value**: 2.0 (optimal for Gaussian distributions)
- **Tuning range**: 0 to 2
- **Effect of increasing**: More weight on the central sigma point for covariance calculation
- **Effect of decreasing**: Less weight on the central sigma point
- **When to adjust**: Generally keep at 2.0 for Gaussian noise; only adjust if you have specific knowledge about the noise distribution

### Kappa (κ)
- **Purpose**: Secondary scaling parameter
- **Default value**: 0.0
- **Tuning range**: 0 to 3-n (where n is state dimension)
- **Effect of increasing**: Increases the spread of sigma points
- **Effect of decreasing**: Can lead to non-positive semi-definite covariance matrices
- **When to adjust**: Only if numerical issues occur; generally keep at 0

## Process Noise Parameters

### Position Noise
- **Purpose**: Accounts for uncertainty in position prediction
- **Default value**: 0.01 (diagonal elements of process noise matrix)
- **Tuning range**: 0.001 to 0.1
- **Effect of increasing**: More responsive to measurements, but potentially more jittery
- **Effect of decreasing**: Smoother trajectory, but potentially slower to respond to changes
- **When to increase**: If the filter is slow to track actual position changes
- **When to decrease**: If position estimates are too noisy

### Orientation Noise
- **Purpose**: Accounts for uncertainty in orientation prediction
- **Default value**: 0.01 (diagonal element of process noise matrix)
- **Tuning range**: 0.001 to 0.1
- **Effect of increasing**: More responsive to orientation changes, but potentially more jittery
- **Effect of decreasing**: Smoother orientation estimates, but potentially slower to respond
- **When to increase**: If the filter is slow to track actual orientation changes
- **When to decrease**: If orientation estimates are too noisy

### Velocity Noise
- **Purpose**: Accounts for uncertainty in velocity prediction
- **Default value**: 0.1 (diagonal elements of process noise matrix)
- **Tuning range**: 0.01 to 1.0
- **Effect of increasing**: More responsive to velocity changes, but potentially more jittery
- **Effect of decreasing**: Smoother velocity estimates, but potentially slower to respond
- **When to increase**: If the filter is slow to track actual velocity changes
- **When to decrease**: If velocity estimates are too noisy

## Measurement Noise Parameters

### Odometry Measurement Noise
- **Purpose**: Accounts for uncertainty in wheel odometry measurements
- **Default value**: 0.01 (diagonal elements of odometry measurement noise matrix)
- **Tuning range**: 0.001 to 0.1
- **Effect of increasing**: Less trust in odometry, more reliance on process model and LiDAR
- **Effect of decreasing**: More trust in odometry, potentially more drift over time
- **When to increase**: If odometry is known to be unreliable or has significant drift
- **When to decrease**: If odometry is highly accurate and reliable

### LiDAR Measurement Noise
- **Purpose**: Accounts for uncertainty in LiDAR range and bearing measurements
- **Default values**: 
  - Range: 0.01 (diagonal element of LiDAR measurement noise matrix)
  - Bearing: 0.01 (diagonal element of LiDAR measurement noise matrix)
- **Tuning range**: 0.001 to 0.1
- **Effect of increasing**: Less trust in LiDAR, more reliance on process model and odometry
- **Effect of decreasing**: More trust in LiDAR, potentially more jumps in position estimates
- **When to increase**: If LiDAR measurements are noisy or unreliable
- **When to decrease**: If LiDAR measurements are highly accurate and reliable

## Systematic Tuning Approach

1. **Start with default values** as specified in the implementation
2. **Test with static position** to ensure filter converges to correct position
3. **Test with simple motion** to ensure filter tracks correctly
4. **Adjust process noise** if tracking is too slow or too jittery
5. **Adjust measurement noise** if filter trusts sensors too much or too little
6. **Fine-tune UKF parameters** if necessary for specific behaviors
7. **Validate with complex trajectories** to ensure overall performance

## Common Issues and Solutions

### Filter Divergence
- **Symptoms**: Estimated position drifts far from true position, covariance grows unbounded
- **Possible causes**: Process noise too small, measurement noise too large, poor initial estimate
- **Solutions**: Increase process noise, decrease measurement noise, improve initialization

### Jumpy Estimates
- **Symptoms**: Position or orientation estimates jump erratically
- **Possible causes**: Process noise too large, measurement noise too small, outliers in measurements
- **Solutions**: Decrease process noise, increase measurement noise, implement outlier rejection

### Slow Response
- **Symptoms**: Filter lags behind actual robot motion
- **Possible causes**: Process noise too small, measurement noise too large
- **Solutions**: Increase process noise, decrease measurement noise

### Poor Accuracy
- **Symptoms**: Consistent offset between estimated and true position
- **Possible causes**: Sensor biases, incorrect sensor models, poor calibration
- **Solutions**: Calibrate sensors, refine measurement models, add bias states to filter

## Recommended Starting Configuration

```cpp
// UKF parameters
double alpha = 0.001;
double beta = 2.0;
double kappa = 0.0;

// Process noise (position, orientation, velocity)
processNoise.topLeftCorner(3, 3) *= 0.01;  // Position and orientation noise
processNoise.bottomRightCorner(3, 3) *= 0.1;  // Velocity noise

// Measurement noise
odomMeasurementNoise = Matrix::Identity(3, 3) * 0.01;  // Odometry measurement noise
lidarMeasurementNoise = Matrix::Identity(2, 2);  // LiDAR measurement noise
lidarMeasurementNoise(0, 0) = 0.01;  // Range noise
lidarMeasurementNoise(1, 1) = 0.01;  // Bearing noise
```

Adjust these values based on the observed performance during testing.
