# RoadRunner Implementation Guide

## Introduction to RoadRunner

RoadRunner is a motion planning library specifically designed for FTC (FIRST Tech Challenge) robots. It provides accurate trajectory generation and following capabilities to help your robot navigate autonomously with precision. This document explains how RoadRunner is implemented in our codebase and how to use it effectively.

## System Overview

Our RoadRunner implementation consists of several integrated components:

1. **Drive System** - Handles the mecanum kinematics and motor control
2. **Localization** - Uses three-wheel odometry for position tracking
3. **Trajectory Generation** - Creates paths for the robot to follow
4. **Trajectory Following** - Ensures the robot accurately follows the generated paths

## Key Components

### 1. Drive Constants

The `DriveConstants.java` file contains all the robot-specific parameters that RoadRunner needs:

```java
public class DriveConstants {
    // Motor specifications
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;
    
    // Robot dimensions
    public static double WHEEL_BASE = 16.0;    // Distance between front and back wheels
    public static double TRACK_WIDTH = 14.0;   // Distance between left and right wheels
    
    // Motion constraints
    public static double MAX_VEL = 30;         // Maximum velocity
    public static double MAX_ACCEL = 30;       // Maximum acceleration
    
    // PID coefficients
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(15, 0, 0.1, 13.8);
    
    // Odometry specifications
    public static double ODOMETRY_WHEEL_RADIUS = 0.75;  // in inches
    public static double ODOMETRY_TICKS_PER_REV = 8192; // REV Through-Bore Encoders
    
    // Odometry wheel positions
    public static double ODOMETRY_LEFT_X = 0.0;   // X position of left odometry wheel
    public static double ODOMETRY_LEFT_Y = 6.0;   // Y position of left odometry wheel
    public static double ODOMETRY_RIGHT_X = 0.0;  // X position of right odometry wheel
    public static double ODOMETRY_RIGHT_Y = -6.0; // Y position of right odometry wheel
    public static double ODOMETRY_MIDDLE_X = -6.0; // X position of middle odometry wheel
    public static double ODOMETRY_MIDDLE_Y = 0.0;  // Y position of middle odometry wheel
}
```

These values should be tuned to match your specific robot. The most critical values are:
- Wheel dimensions and encoder counts
- Robot dimensions (WHEEL_BASE and TRACK_WIDTH)
- Odometry wheel positions relative to robot center
- PID coefficients for motion control

### 2. Mecanum Drive Implementation

The `SampleMecanumDrive.java` class extends RoadRunner's `MecanumDrive` and handles:
- Motor initialization and configuration
- Trajectory generation and following
- Odometry integration

```java
public class SampleMecanumDrive extends MecanumDrive {
    // Motor declarations
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    
    // RoadRunner components
    private final TrajectoryFollower follower;
    private final TrajectorySequenceRunner trajectorySequenceRunner;
    
    // Constructor
    public SampleMecanumDrive(HardwareMap hardwareMap) {
        // Initialize drive system
        // Initialize trajectory followers
        // Configure motors
    }
    
    // Methods for trajectory generation and following
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) { ... }
    public void followTrajectory(Trajectory trajectory) { ... }
    public void followTrajectorySequence(TrajectorySequence sequence) { ... }
}
```

This class acts as the central hub for all RoadRunner functionality.

### 3. Three-Wheel Odometry

The `ThreeWheelLocalizer.java` class provides position tracking using three "dead wheel" encoders:

```java
public class ThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
    // Encoder objects
    private final DcMotorEx leftEncoder, rightEncoder, strafeEncoder;
    
    // Conversion factors
    public static double TICKS_PER_INCH = TICKS_PER_REV / (2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO);
    
    // Positions and tracking methods
    @Override
    public List<Double> getWheelPositions() { ... }
    
    @Override
    public List<Double> getWheelVelocities() { ... }
}
```

This class:
- Reads encoder values from the three tracking wheels
- Converts encoder ticks to distances
- Calculates the robot's position and heading
- Provides this information to the drive system

## Odometry Setup

Our implementation uses a three-wheel odometry configuration:

1. **Left Encoder** - Parallel to the robot's forward direction, positioned on the left side
2. **Right Encoder** - Parallel to the robot's forward direction, positioned on the right side
3. **Strafe/Middle Encoder** - Perpendicular to the robot's forward direction (measures lateral movement)

This setup provides:
- X position tracking (forward/backward)
- Y position tracking (left/right)
- Heading tracking (rotation)

The positions of these wheels are defined in `DriveConstants.java` and used by `ThreeWheelLocalizer`.

## Trajectory Generation and Following

### Building Simple Trajectories

```java
// Create a drive instance
SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

// Set the starting position
Pose2d startPose = new Pose2d(0, 0, 0);
drive.setPoseEstimate(startPose);

// Build a trajectory
Trajectory trajectory = drive.trajectoryBuilder(startPose)
    .lineTo(new Vector2d(24, 24))  // Move 24 inches forward and right
    .build();

// Follow the trajectory
drive.followTrajectory(trajectory);
```

### Building Complex Trajectory Sequences

```java
// Build a complex sequence
TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
    // Move forward with a spline path
    .splineTo(new Vector2d(30, 30), Math.toRadians(0))
    
    // Turn 90 degrees
    .turn(Math.toRadians(90))
    
    // Wait for 2 seconds
    .waitSeconds(2)
    
    // Move to another position
    .lineTo(new Vector2d(0, 60))
    
    // Build the sequence
    .build();

// Follow the sequence
drive.followTrajectorySequence(sequence);
```

## Path Types

RoadRunner supports several types of paths:

1. **Line Segments** - Direct straight-line motion
   ```java
   .lineTo(new Vector2d(x, y))
   ```

2. **Spline Paths** - Smooth curved motion
   ```java
   .splineTo(new Vector2d(x, y), heading)
   ```

3. **Heading Control Options**
   - Constant heading (robot orientation remains fixed)
     ```java
     .lineToConstantHeading(new Vector2d(x, y))
     .splineToConstantHeading(new Vector2d(x, y), tangent)
     ```
   
   - Linear heading (robot orientation changes linearly)
     ```java
     .lineToLinearHeading(new Pose2d(x, y, heading))
     .splineToLinearHeading(new Pose2d(x, y, heading), tangent)
     ```

4. **Turn in Place**
   ```java
   .turn(angleRadians)
   ```

5. **Wait**
   ```java
   .waitSeconds(seconds)
   ```

## The Coordinate System

RoadRunner uses a field-centric coordinate system:
- X-axis: Forward/backward (relative to starting position)
- Y-axis: Left/right (relative to starting position)
- Heading: Counter-clockwise rotation in radians (0 = forward)

## Sequence Execution Flow

When you call `drive.followTrajectorySequence(sequence)`, the following happens:

1. `trajectorySequenceRunner` begins executing the first segment
2. In the `update()` method:
   - Position is estimated from odometry
   - Current segment is executed based on its type
     - For trajectory segments, the `follower` handles tracking
     - For turn segments, the `turnController` handles rotation
     - For wait segments, time passes but no motion occurs
   - When a segment completes, the next one begins automatically
3. The sequence continues until all segments are completed

## Tuning and Optimization

For optimal performance, you need to tune:

1. **Drive Constants** - Make sure robot dimensions and constraints are accurate

2. **PID Coefficients** - Adjust for smooth, accurate following:
   - Translational PID - Controls forward/sideways motion
   - Heading PID - Controls rotational motion
   - Drive PID - Controls individual wheel velocities

3. **Odometry Configuration** - Accurately measure and configure:
   - Wheel radius and encoder counts
   - Wheel positions relative to robot center
   - Encoder directions (forward/reverse)

## Example: Triangle Autonomous Path

The `RoadRunnerTriangle.java` example demonstrates:

```java
private TrajectorySequence buildTriangleSequence() {
    return drive.trajectorySequenceBuilder(startPose)
            // First segment: start to intermediate
            .splineToLinearHeading(interPose, Math.toRadians(0))
            
            // Second segment: intermediate to end
            .splineToLinearHeading(endPose, Math.toRadians(90))
            
            // Third segment: end back to start
            .splineToLinearHeading(startPose, Math.toRadians(225))
            
            .build();
}
```

This creates a smooth triangular path:
1. Start at (0,0) with 0° heading
2. Move to (24,-24) and rotate to 90° heading
3. Move to (24,24) and rotate to 45° heading
4. Return to (0,0) and rotate to 0° heading

## Troubleshooting

Common issues and solutions:

1. **Poor tracking accuracy**
   - Verify odometry wheel positions are correct
   - Check encoder directions
   - Tune PID coefficients

2. **Jerky motion**
   - Reduce MAX_VEL and MAX_ACCEL
   - Tune PID coefficients

3. **Robot not reaching target**
   - Check for mechanical issues
   - Verify encoder functionality
   - Increase PID tolerances

## Conclusion

RoadRunner provides a powerful framework for precise robot movement. By properly configuring and tuning the system, you can achieve accurate autonomous navigation. The code structure allows for easy creation of complex paths while handling the mathematical complexity for you.

For further tuning and information, refer to the official RoadRunner documentation.