# Unscented Kalman Filter (UKF) Theory for Robot Localization

## Overview
The Unscented Kalman Filter (UKF) is an advanced state estimation technique that addresses the limitations of the Extended Kalman Filter (EKF) when dealing with highly nonlinear systems. This document outlines the theoretical foundation of UKF specifically for robot localization applications.

## Mathematical Foundation

### State Representation
For a three-wheeled omniwheel robot, the state vector typically includes:
- Position (x, y)
- Orientation (θ)
- Linear velocities (vx, vy)
- Angular velocity (ω)

State vector: x = [x, y, θ, vx, vy, ω]ᵀ

### Unscented Transform
The core of UKF is the unscented transform, which uses a set of carefully chosen sample points (sigma points) to represent the probability distribution.

1. **Sigma Points Generation**:
   For an n-dimensional state vector, 2n+1 sigma points are generated:
   - X₀ = x̄ (mean state)
   - Xᵢ = x̄ + (√((n+λ)P))ᵢ for i = 1,...,n
   - Xᵢ₊ₙ = x̄ - (√((n+λ)P))ᵢ₋ₙ for i = n+1,...,2n
   
   Where:
   - λ = α²(n+κ) - n is a scaling parameter
   - α determines the spread of sigma points (typically 10⁻⁴ ≤ α ≤ 1)
   - κ is a secondary scaling parameter (typically κ = 0 or 3-n)
   - P is the state covariance matrix
   - (√((n+λ)P))ᵢ is the ith column of the matrix square root

2. **Weights Calculation**:
   - W₀ᵐ = λ/(n+λ)
   - W₀ᶜ = λ/(n+λ) + (1-α²+β)
   - Wᵢᵐ = Wᵢᶜ = 1/(2(n+λ)) for i = 1,...,2n
   
   Where:
   - W₀ᵐ is the weight for the mean of the central sigma point
   - W₀ᶜ is the weight for the covariance of the central sigma point
   - β is used to incorporate prior knowledge (β = 2 is optimal for Gaussian distributions)

### UKF Algorithm for Robot Localization

#### Initialization
1. Initialize state estimate x̂₀ and covariance P₀
2. Set UKF parameters α, β, and κ

#### Prediction Step
1. Generate sigma points Xₖ₋₁ based on previous state estimate x̂ₖ₋₁ and covariance Pₖ₋₁
2. Propagate sigma points through the motion model:
   - X̃ₖ = f(Xₖ₋₁, uₖ₋₁) where f is the motion model and uₖ₋₁ is the control input
3. Calculate predicted mean and covariance:
   - x̂ₖ⁻ = ∑ᵢ Wᵢᵐ X̃ₖᵢ
   - Pₖ⁻ = ∑ᵢ Wᵢᶜ (X̃ₖᵢ - x̂ₖ⁻)(X̃ₖᵢ - x̂ₖ⁻)ᵀ + Qₖ₋₁
   
   Where Qₖ₋₁ is the process noise covariance

#### Correction Step
1. Generate new sigma points X̃ₖ based on predicted state x̂ₖ⁻ and covariance Pₖ⁻
2. Transform sigma points through measurement model:
   - Ỹₖ = h(X̃ₖ) where h is the measurement model
3. Calculate predicted measurement mean and covariance:
   - ŷₖ = ∑ᵢ Wᵢᵐ Ỹₖᵢ
   - Pyy = ∑ᵢ Wᵢᶜ (Ỹₖᵢ - ŷₖ)(Ỹₖᵢ - ŷₖ)ᵀ + Rₖ
   
   Where Rₖ is the measurement noise covariance
4. Calculate cross-correlation matrix:
   - Pxy = ∑ᵢ Wᵢᶜ (X̃ₖᵢ - x̂ₖ⁻)(Ỹₖᵢ - ŷₖ)ᵀ
5. Calculate Kalman gain:
   - Kₖ = Pxy Pyy⁻¹
6. Update state estimate and covariance:
   - x̂ₖ = x̂ₖ⁻ + Kₖ(zₖ - ŷₖ)
   - Pₖ = Pₖ⁻ - Kₖ Pyy Kₖᵀ
   
   Where zₖ is the actual measurement

## Motion Model for Omniwheel Robot
For a three-wheeled omniwheel robot, the motion model is nonlinear due to the holonomic nature of the platform:

f(xₖ₋₁, uₖ₋₁) = [
    xₖ₋₁ + (vx,ₖ₋₁ cos(θₖ₋₁) - vy,ₖ₋₁ sin(θₖ₋₁)) Δt,
    yₖ₋₁ + (vx,ₖ₋₁ sin(θₖ₋₁) + vy,ₖ₋₁ cos(θₖ₋₁)) Δt,
    θₖ₋₁ + ωₖ₋₁ Δt,
    vx,ₖ₋₁ + ax,ₖ₋₁ Δt,
    vy,ₖ₋₁ + ay,ₖ₋₁ Δt,
    ωₖ₋₁ + αₖ₋₁ Δt
]

Where:
- Δt is the time step
- ax,ₖ₋₁, ay,ₖ₋₁, αₖ₋₁ are the accelerations derived from control inputs

## Measurement Models

### Wheel Odometry Measurement Model
The wheel odometry provides velocity measurements:

h_odom(xₖ) = [vx,ₖ, vy,ₖ, ωₖ]ᵀ

### LiDAR Measurement Model
The LiDAR provides range and bearing measurements to landmarks:

h_lidar(xₖ, mⱼ) = [
    √((mⱼ,x - xₖ)² + (mⱼ,y - yₖ)²),
    atan2(mⱼ,y - yₖ, mⱼ,x - xₖ) - θₖ
]

Where mⱼ = [mⱼ,x, mⱼ,y]ᵀ is the position of landmark j.

## Sensor Fusion Strategy
UKF naturally handles multi-rate sensor fusion by:
1. Performing prediction steps at the highest sensor rate (typically odometry)
2. Performing correction steps whenever measurements are available
3. Using appropriate measurement models based on the sensor type

## Advantages for Eurobot Application
1. **Accurate Nonlinear Estimation**: UKF better captures the nonlinear dynamics of omnidirectional robots
2. **No Jacobian Calculation**: Avoids complex derivative calculations required by EKF
3. **Robust to Initial Errors**: More likely to converge from poor initial estimates
4. **Better Uncertainty Representation**: More accurately represents the true probability distribution
5. **Efficient Sensor Fusion**: Naturally handles multi-rate, multi-sensor fusion

## Implementation Considerations
1. **Parameter Tuning**: α, β, and κ parameters need careful tuning
2. **Computational Efficiency**: Optimize sigma point calculations for real-time performance
3. **Numerical Stability**: Use square-root UKF for improved numerical stability in covariance calculations
4. **Adaptive Process Noise**: Consider adaptive process noise to handle varying dynamics
5. **Consistency Checking**: Implement consistency checks to detect filter divergence
