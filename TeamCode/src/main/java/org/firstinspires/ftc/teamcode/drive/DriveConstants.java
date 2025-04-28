package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Constants shared between multiple drive types.
 * 
 * TODO: Tune these constants to fit your robot!
 */
@Config
public class DriveConstants {

    /**
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;
    
    /**
     * Motor gear reduction and wheel radius
     */
    public static final double GEAR_RATIO = 1.0; // output (wheel) speed / input (motor) speed
    public static final double WHEEL_RADIUS = 1.88976;  // in inches

    /**
     * Set RUN_USING_ENCODER to true to enable built-in hub motor velocity control.
     * Set this to false if drive encoders aren't connected.
     * 
     * If using the REV Hub, this value should be true.
     * Otherwise, false is recommended.
     */
    public static final boolean RUN_USING_ENCODER = true;

    /**
     * Set the orientation of the motors based on your robot configuration
     * This ensures motors are correctly configured for forward/reverse
     */
    public static final boolean REVERSED_FRONT_LEFT = true;
    public static final boolean REVERSED_FRONT_RIGHT = false;
    public static final boolean REVERSED_BACK_LEFT = true;
    public static final boolean REVERSED_BACK_RIGHT = false;

    /**
     * These constants define the center-to-wheel distances and are used for calculating wheel powers
     * during mecanum drive calculations. Make sure to update these with the actual measurements (in inches)
     * from your robot.
     */
    public static double WHEEL_BASE = 16.25; // Distance between left and right wheels
    public static double TRACK_WIDTH = 9.5; // Distance between front and back wheels

    /**
     * These values are used to generate the trajectories for your robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities.
     */
    public static double MAX_VEL = 52.48180821614297; // 30;
    public static double MAX_ACCEL = 52.48180821614297; // 30;
    public static double MAX_ANG_VEL = Math.toRadians(184.02607784577722); // Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(184.02607784577722); // Math.toRadians(180);

    /**
     * PID Coefficients for RoadRunner closed-loop control.
     * For drive motors (controls velocity)
     */
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(15, 0, 0.1, 13.8);

    /**
     * PID Coefficients for the translational motion controller
     */
    public static double TRANSLATIONAL_PID_COEFF_P = 8.0;
    public static double TRANSLATIONAL_PID_COEFF_I = 0.0;
    public static double TRANSLATIONAL_PID_COEFF_D = 0.0;

    /**
     * PID Coefficients for the heading motion controller
     */
    public static double HEADING_PID_COEFF_P = 8.0;
    public static double HEADING_PID_COEFF_I = 0.0;
    public static double HEADING_PID_COEFF_D = 0.0;

    /**
     * These are physical constraints of the robot that affect odometry
     */
    public static double LATERAL_MULTIPLIER = 1.0; // Multiplier for lateral vs forward movement
    
    /**
     * Odometry wheel radius (assumed all three wheels have the same radius)
     */
    public static double ODOMETRY_WHEEL_RADIUS = 0.945; // in inches
    
    /**
     * Encoder counts per rotation for odometry pods
     */
    public static double ODOMETRY_TICKS_PER_REV = 2000; // REV Through-Bore Encoders
    
    /**
     * Positions of the three odometry wheels relative to the center of the robot
     * These values are critical for accurate odometry
     * They are the distances in inches from the center of the robot to each wheel
     */
    public static double ODOMETRY_LEFT_X = -1.0;  // X position of the left odometry wheel
    public static double ODOMETRY_LEFT_Y = 8.125;  // Y position of the left odometry wheel
    
    public static double ODOMETRY_RIGHT_X = -0.75;  // X position of the right odometry wheel
    public static double ODOMETRY_RIGHT_Y = -7.875; // Y position of the right odometry wheel
    
    public static double ODOMETRY_MIDDLE_X = -6.625; // X position of the middle/strafe odometry wheel
    public static double ODOMETRY_MIDDLE_Y = -1.0;  // Y position of the middle/strafe odometry wheel
}