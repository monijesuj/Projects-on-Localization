# UKF Design for Three-Wheeled Omniwheel Robot Localization

## State Space Model

### State Vector
For our three-wheeled omniwheel robot, we'll use the following state vector:

```
x = [x, y, θ, vx, vy, ω]ᵀ
```

Where:
- `x, y`: Position in global coordinates (m)
- `θ`: Orientation (rad)
- `vx, vy`: Linear velocities in robot frame (m/s)
- `ω`: Angular velocity (rad/s)

This 6-dimensional state vector captures both the pose and the dynamics of the robot, allowing for accurate prediction of motion.

### Covariance Matrix
The state covariance matrix P is a 6×6 symmetric positive definite matrix representing the uncertainty in the state estimate.

## Process Model

### Motion Model
The process model predicts how the state evolves over time based on the current state and control inputs:

```
x_{k+1} = f(x_k, u_k) + w_k
```

For our omniwheel robot, the motion model is:

```
x_{k+1} = [
    x_k + (vx_k*cos(θ_k) - vy_k*sin(θ_k))*dt,
    y_k + (vx_k*sin(θ_k) + vy_k*cos(θ_k))*dt,
    θ_k + ω_k*dt,
    vx_k,
    vy_k,
    ω_k
]
```

Where:
- `dt`: Time step between updates
- `w_k`: Process noise, assumed to be zero-mean Gaussian with covariance Q

### Process Noise Covariance
The process noise covariance matrix Q accounts for uncertainties in the motion model:

```
Q = diag([σ²_x, σ²_y, σ²_θ, σ²_vx, σ²_vy, σ²_ω])
```

These values will be tuned based on the wheel odometry characteristics observed in the existing implementation.

## Measurement Models

### Wheel Odometry Measurement Model
The wheel odometry provides velocity measurements in the robot frame:

```
z_odom = [vx, vy, ω]ᵀ + v_odom
```

The measurement model is:

```
h_odom(x) = [vx, vy, ω]ᵀ
```

Where `v_odom` is the measurement noise, assumed to be zero-mean Gaussian with covariance R_odom.

### LiDAR Measurement Model
The LiDAR provides range and bearing measurements to landmarks (beacons):

```
z_lidar = [r_1, φ_1, r_2, φ_2, ..., r_n, φ_n]ᵀ + v_lidar
```

For each landmark j, the measurement model is:

```
h_lidar(x, m_j) = [
    sqrt((m_j,x - x)² + (m_j,y - y)²),
    atan2(m_j,y - y, m_j,x - x) - θ
]
```

Where:
- `m_j = [m_j,x, m_j,y]ᵀ`: Position of landmark j
- `v_lidar`: Measurement noise, assumed to be zero-mean Gaussian with covariance R_lidar

## UKF Algorithm Implementation

### Initialization
1. Initialize state estimate x̂₀ based on the starting position (similar to particle filter)
2. Initialize covariance P₀ based on initial uncertainty
3. Set UKF parameters:
   - α = 0.001 (determines spread of sigma points, small value for accurate estimation)
   - β = 2 (optimal for Gaussian distributions)
   - κ = 0 (secondary scaling parameter)
   - λ = α²(n+κ)-n (scaling parameter, where n is state dimension)

### Prediction Step (Time Update)
1. Generate sigma points based on current state estimate and covariance
2. Propagate sigma points through the motion model
3. Calculate predicted mean and covariance

### Correction Step (Measurement Update)
1. Generate new sigma points based on predicted state and covariance
2. Transform sigma points through the appropriate measurement model
3. Calculate predicted measurement mean and covariance
4. Calculate cross-correlation matrix
5. Calculate Kalman gain
6. Update state estimate and covariance

### Multi-Sensor Fusion Strategy
1. Perform prediction steps at the highest sensor rate (typically odometry)
2. Perform correction steps whenever measurements are available
3. Use appropriate measurement models based on the sensor type

## Integration with Existing Code Structure

### UKF Class Structure
We'll create a `UnscentedKalmanFilter` class that follows a similar interface to the existing `ParticleFilter` class:

```cpp
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
        const Points& landmarks);

    Gaussian localize(double referenceTime, const Points& beacons);
    
private:
    void generateSigmaPoints();
    void predictState(double dt);
    void updateWithOdometry(const SensorData& odomData);
    void updateWithLidar(const Points& beacons);
    
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
    Points landmarks;
    Pose prevPose;
};
```

### Integration with Localization Node
The `LocalizationNode` class will be modified to use the `UnscentedKalmanFilter` instead of the `ParticleFilter`:

1. Replace the `ParticleFilter` instance with `UnscentedKalmanFilter`
2. Keep the same sensor interfaces and callbacks
3. Update the publishing methods to use the UKF output

## Parameter Tuning Strategy

### Process Noise Parameters
- Start with conservative values based on the existing particle filter noise parameters
- Gradually reduce process noise as the filter stabilizes
- Tune separately for position, orientation, and velocity components

### Measurement Noise Parameters
- For wheel odometry: Use the existing noise model from the `WheelOdom` sensor class
- For LiDAR: Use the existing noise model from the `LiDAR` sensor class
- Adjust based on observed performance

### UKF Specific Parameters
- α: Start with 0.001 (small value for accurate estimation)
- β: Use 2 (optimal for Gaussian distributions)
- κ: Use 0 (standard value)

## Performance Considerations

### Computational Efficiency
- Implement matrix operations efficiently using Eigen library
- Consider using square-root UKF for improved numerical stability
- Optimize sigma point calculations for real-time performance

### Robustness
- Implement consistency checks to detect filter divergence
- Consider adaptive process noise to handle varying dynamics
- Implement outlier rejection for LiDAR measurements

### Accuracy
- Target 1cm or better position accuracy
- Implement smoothing techniques for improved accuracy
- Consider using more sigma points for critical operations
