# UKF Testing and Validation

This document outlines the testing and validation approach for the Unscented Kalman Filter (UKF) implementation for the three-wheeled omniwheel robot in the Eurobot competition.

## Unit Testing

A unit test file (`ukf_test.cpp`) has been created to validate the core functionality of the UKF implementation. This test:

1. Creates a simulated environment with landmarks
2. Initializes the UKF with appropriate parameters
3. Simulates sensor data (wheel odometry and LiDAR)
4. Runs the localization algorithm
5. Verifies that the results are reasonable and mathematically sound

The test checks:
- That the state updates correctly in response to motion
- That the covariance matrix remains positive definite
- That the filter behaves as expected with simulated measurements

## Integration Testing

To fully validate the UKF implementation in the actual robot environment, the following tests should be performed:

### 1. Static Position Test
- Place the robot at a known position
- Run the localization with the robot stationary
- Verify that the estimated position matches the known position
- Check that the covariance converges to a small value

### 2. Simple Motion Test
- Move the robot in a straight line
- Verify that the estimated trajectory follows the actual path
- Check that the position error remains within the 1cm target

### 3. Rotation Test
- Rotate the robot in place
- Verify that the estimated orientation changes correctly
- Check that the position estimate remains stable

### 4. Complex Trajectory Test
- Move the robot in a complex pattern (e.g., figure-eight)
- Verify that the estimated trajectory matches the actual path
- Check that the position error remains within the 1cm target

### 5. Sensor Failure Test
- Temporarily disable one sensor (e.g., LiDAR)
- Verify that the filter continues to function with degraded performance
- Re-enable the sensor and check that performance recovers

### 6. Comparison with Particle Filter
- Run both UKF and particle filter simultaneously
- Compare the estimated positions and covariances
- Evaluate which filter provides better accuracy and robustness

## Performance Metrics

The following metrics should be used to evaluate the UKF performance:

### 1. Accuracy
- Position error (target: < 1cm)
- Orientation error (target: < 1 degree)
- Consistency of error (standard deviation)

### 2. Computational Efficiency
- Processing time per update cycle
- Memory usage
- CPU load

### 3. Robustness
- Recovery from sensor glitches
- Performance with partial sensor data
- Convergence from poor initial estimates

## Tuning Guidelines

If the UKF performance needs improvement, consider adjusting the following parameters:

### 1. UKF Parameters
- `alpha`: Controls the spread of sigma points (decrease for more accurate linearization)
- `beta`: Incorporates prior knowledge about the distribution (2 is optimal for Gaussian)
- `kappa`: Secondary scaling parameter (adjust if numerical issues occur)

### 2. Process Noise
- Increase for more responsive tracking but higher noise
- Decrease for smoother tracking but slower response

### 3. Measurement Noise
- Increase for more trust in the process model
- Decrease for more trust in the measurements

## Expected Results

A properly implemented and tuned UKF should provide:
- Position accuracy better than 1cm in normal operation
- Smooth and consistent tracking during complex maneuvers
- Better handling of nonlinearities compared to EKF
- Comparable or better performance than the particle filter with fewer computational resources
