# Extended Kalman Filter (EKF) SLAM Implementation

## Overview
This MATLAB script implements EKF-SLAM (Simultaneous Localization and Mapping) for a robot observing a known landmark. It demonstrates real-time state estimation and uncertainty handling.

## Features
- Real-time EKF-SLAM implementation
- Known landmark observation
- State estimation with covariance tracking
- Visualization of robot and landmark positions

## Parameters
- State vector: [x, y, theta]
- Initial covariance: Identity matrix
- Process noise (Q): 0.01 * I
- Measurement noise (R): 0.1 * I
- Control inputs: Linear and angular velocity
- Runtime: 30 seconds
- Time step: 0.1 seconds

## Components
- Prediction step with boundary checking
- Measurement simulation (range and bearing)
- EKF update step
- Real-time visualization

## Usage
1. Configure initial parameters
2. Run the script
3. Observe robot movement and state estimation

## Visualization
- Blue dots: Robot position
- Red x: Landmark position
- Grid: [-10, 10] x [-10, 10]
- ![EKF](https://github.com/user-attachments/assets/3b3ab1dd-d741-43eb-a27d-a1c2fd3aac38)
