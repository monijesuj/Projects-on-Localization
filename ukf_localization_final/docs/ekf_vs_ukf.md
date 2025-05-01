# EKF vs UKF for Robot Localization in Eurobot Competition

## Introduction
This document compares Extended Kalman Filter (EKF) and Unscented Kalman Filter (UKF) for robot localization in the context of the Eurobot competition, specifically for a three-wheeled omniwheel robot using wheel odometry and Hokuyo LiDAR sensors.

## Kalman Filter Basics
Both EKF and UKF are variants of the Kalman Filter, which is a recursive state estimator that works in two phases:
1. **Prediction**: Uses the motion model to predict the next state
2. **Correction**: Uses sensor measurements to correct the prediction

The key difference between variants is how they handle nonlinearities in the system.

## Extended Kalman Filter (EKF)

### Advantages
- Computationally efficient compared to UKF
- Well-established and widely used in robotics
- Simpler implementation
- Lower memory requirements
- Faster execution time, which can be beneficial for real-time applications

### Disadvantages
- Relies on first-order linearization of nonlinear functions
- Performance degrades with highly nonlinear systems
- Requires calculation of Jacobian matrices, which can be complex and error-prone
- May diverge if the linearization is not a good approximation

### Suitability for Eurobot
- Works well when the robot's motion is relatively smooth and predictable
- Adequate for systems where linearization provides a good approximation
- May struggle with the omnidirectional motion of a three-wheeled robot, especially during rapid direction changes

## Unscented Kalman Filter (UKF)

### Advantages
- Handles nonlinearities better through the unscented transform
- Does not require calculation of Jacobian matrices
- More accurate for highly nonlinear systems
- More robust against initial estimation errors
- Better captures the true mean and covariance of the transformed distribution

### Disadvantages
- More computationally intensive than EKF
- Requires more memory for storing sigma points
- Slightly more complex implementation
- May have higher latency, though this is less significant on modern hardware like an Intel NUC

### Suitability for Eurobot
- Better handles the complex dynamics of omnidirectional robots
- More accurate for rapid direction changes and complex maneuvers
- Can better integrate multiple sensor types with different noise characteristics
- Better maintains accuracy during aggressive maneuvers

## Considerations for Three-Wheeled Omniwheel Robot

### Motion Model
- Omnidirectional robots have nonlinear kinematics, especially during rapid direction changes
- UKF can better capture these nonlinearities without linearization
- The holonomic nature of the robot introduces complexities that UKF handles more gracefully

### Sensor Integration
- Wheel odometry provides high-frequency but drift-prone data
- Hokuyo LiDAR provides accurate but lower-frequency measurements
- UKF can better fuse these complementary sensors with different update rates and noise characteristics

### Accuracy Requirements
- The target accuracy of 1cm or less is demanding
- UKF typically achieves better accuracy in nonlinear systems
- The improved handling of nonlinearities can help maintain accuracy during complex maneuvers

### Computational Resources
- Intel NUC provides sufficient computational power for either approach
- The slight increase in computational demand for UKF is easily handled by modern processors
- Real-time performance is achievable with either method on this hardware

## Recommendation

Based on the analysis, the **Unscented Kalman Filter (UKF)** is recommended for this application for the following reasons:

1. **Better handling of nonlinearities**: The omnidirectional motion of a three-wheeled robot introduces significant nonlinearities that UKF handles more effectively.

2. **No need for Jacobian matrices**: UKF avoids the potentially complex and error-prone calculation of Jacobian matrices.

3. **Superior sensor fusion**: UKF better integrates the complementary nature of wheel odometry (high-frequency, drift-prone) and LiDAR (accurate but lower-frequency).

4. **Higher accuracy**: UKF typically achieves better accuracy in nonlinear systems, helping meet the demanding 1cm accuracy requirement.

5. **Sufficient computational resources**: The Intel NUC provides ample processing power to handle the additional computational demands of UKF.

6. **Robustness**: UKF is more robust to initial estimation errors and can better maintain accuracy during aggressive maneuvers common in competition scenarios.

While EKF could be a viable alternative if computational resources were more constrained, the benefits of UKF in terms of accuracy and robustness outweigh its slightly higher computational cost, especially given the available hardware and the demanding accuracy requirements.
