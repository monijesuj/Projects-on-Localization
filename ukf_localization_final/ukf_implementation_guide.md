# UKF Localization Implementation Guide

This document provides a comprehensive guide to the Unscented Kalman Filter (UKF) localization implementation for the three-wheeled omniwheel robot in the Eurobot competition.

## Overview

The UKF localization system is designed to provide high-accuracy position and orientation estimates for a three-wheeled omniwheel robot using wheel odometry and Hokuyo LiDAR sensors. The implementation is based on the existing particle filter framework but offers improved handling of nonlinear dynamics and more accurate state estimation.

## Features

- 6-dimensional state vector tracking position, orientation, and velocities
- Nonlinear motion model for omnidirectional robot kinematics
- Sensor fusion between wheel odometry and LiDAR measurements
- Configurable parameters for fine-tuning performance
- Seamless integration with existing ROS2 framework
- Option to switch between UKF and particle filter

## File Structure

```
localization/
├── include/localization/
│   ├── localization_node.hpp  (Modified to support UKF)
│   ├── sensor.hpp             (Unchanged, used by UKF)
│   ├── tools.hpp              (Unchanged, used by UKF)
│   ├── types.hpp              (Modified to add Matrix type)
│   ├── ukf.hpp                (New, UKF class definition)
│   └── ...
├── src/
│   ├── localization_node.cpp  (Modified to support UKF)
│   ├── ukf.cpp                (New, UKF implementation)
│   ├── ukf_test.cpp           (New, UKF test program)
│   └── ...
```

## Implementation Details

### State Vector

The UKF uses a 6-dimensional state vector:
```
x = [x, y, θ, vx, vy, ω]ᵀ
```

Where:
- `x, y`: Position in global coordinates (m)
- `θ`: Orientation (rad)
- `vx, vy`: Linear velocities in robot frame (m/s)
- `ω`: Angular velocity (rad/s)

### Motion Model

The motion model predicts how the state evolves over time:

```cpp
// Motion model for omnidirectional robot
double cos_theta = std::cos(theta);
double sin_theta = std::sin(theta);

// Predict using motion model
predictedSigmaPoints(0, i) = x + (vx * cos_theta - vy * sin_theta) * dt;
predictedSigmaPoints(1, i) = y + (vx * sin_theta + vy * cos_theta) * dt;
predictedSigmaPoints(2, i) = theta + omega * dt;
predictedSigmaPoints(3, i) = vx;  // Assume constant velocity
predictedSigmaPoints(4, i) = vy;  // Assume constant velocity
predictedSigmaPoints(5, i) = omega;  // Assume constant angular velocity
```

### Measurement Models

#### Wheel Odometry
The wheel odometry provides velocity measurements in the robot frame:

```cpp
// Update velocities in state vector based on wheel odometry
stateVector(3) = displacement(0);  // vx
stateVector(4) = displacement(1);  // vy
stateVector(5) = displacement(2);  // omega
```

#### LiDAR
The LiDAR provides range and bearing measurements to landmarks (beacons):

```cpp
// Calculate expected measurement (range and bearing)
double dx = closestLandmark(0) - x;
double dy = closestLandmark(1) - y;

double range = std::sqrt(dx*dx + dy*dy);
double bearing = std::atan2(dy, dx) - theta;
```

### UKF Algorithm

The UKF algorithm follows the standard steps:

1. **Initialization**: Set initial state and covariance
2. **Sigma Points Generation**: Generate sigma points around current state
3. **Prediction**: Propagate sigma points through motion model
4. **Measurement Update**: Update state based on sensor measurements
5. **Repeat**: Continue the cycle for each new measurement

## Configuration Parameters

### UKF Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ukf_alpha` | 0.001 | Controls spread of sigma points |
| `ukf_beta` | 2.0 | Incorporates prior knowledge (optimal for Gaussian) |
| `ukf_kappa` | 0.0 | Secondary scaling parameter |
| `use_ukf` | true | Whether to use UKF (true) or particle filter (false) |

### Process Noise Parameters

Process noise is configured in the UKF constructor:

```cpp
processNoise.topLeftCorner(3, 3) *= 0.01;  // Position and orientation noise
processNoise.bottomRightCorner(3, 3) *= 0.1;  // Velocity noise
```

### Measurement Noise Parameters

Measurement noise is configured in the UKF constructor:

```cpp
odomMeasurementNoise = Matrix::Identity(3, 3) * 0.01;  // Odometry measurement noise
lidarMeasurementNoise = Matrix::Identity(2, 2);  // LiDAR measurement noise
lidarMeasurementNoise(0, 0) = 0.01;  // Range noise
lidarMeasurementNoise(1, 1) = 0.01;  // Bearing noise
```

## Usage

### Enabling UKF

The UKF is enabled by default. To switch between UKF and particle filter, set the `use_ukf` parameter:

```bash
# Enable UKF (default)
ros2 param set /localization_node use_ukf true

# Enable particle filter
ros2 param set /localization_node use_ukf false
```

### Tuning UKF Parameters

To tune the UKF parameters:

```bash
# Set alpha parameter
ros2 param set /localization_node ukf_alpha 0.001

# Set beta parameter
ros2 param set /localization_node ukf_beta 2.0

# Set kappa parameter
ros2 param set /localization_node ukf_kappa 0.0
```

### Running the Test Program

A test program is provided to validate the UKF implementation:

```bash
# Build the test program
colcon build --packages-select localization

# Run the test
ros2 run localization ukf_test
```

## Performance Considerations

### Computational Efficiency

The UKF is more computationally efficient than the particle filter for similar accuracy levels. However, it still requires significant computation for the sigma point transformations. The implementation is optimized for the Intel NUC hardware.

### Accuracy

The UKF is designed to achieve position accuracy of 1cm or better. This is achieved through:
- Accurate motion model for omnidirectional kinematics
- Proper handling of nonlinearities through the unscented transform
- Effective sensor fusion between wheel odometry and LiDAR

### Robustness

The UKF implementation includes several robustness features:
- Fallback sensor mechanism for handling sensor failures
- Covariance consistency checks
- Angle wrapping to handle orientation discontinuities

## Troubleshooting

### Filter Divergence

If the filter diverges (estimated position drifts far from true position):
- Increase process noise
- Decrease measurement noise
- Check sensor calibration

### Jumpy Estimates

If position or orientation estimates jump erratically:
- Decrease process noise
- Increase measurement noise
- Implement outlier rejection

### Slow Response

If the filter lags behind actual robot motion:
- Increase process noise
- Decrease measurement noise

## Further Improvements

Potential future improvements to the UKF implementation:
- Adaptive process noise based on robot dynamics
- Square-root UKF for improved numerical stability
- Integration with visual odometry for enhanced accuracy
- Online parameter tuning based on performance metrics

## References

- Julier, S.J. and Uhlmann, J.K. (2004). Unscented filtering and nonlinear estimation. Proceedings of the IEEE, 92(3), 401-422.
- Wan, E.A. and Van Der Merwe, R. (2000). The unscented Kalman filter for nonlinear estimation. In Proceedings of the IEEE 2000 Adaptive Systems for Signal Processing, Communications, and Control Symposium (pp. 153-158).
