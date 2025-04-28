package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/**
 * Three-wheel dead wheel localizer implementation. 
 * The localizer uses three odometry wheels positioned in a standard configuration 
 * (two parallel and one perpendicular) to track the robot's position on the field.
 */
@Config
public class ThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
    // Odometry wheel encoder names
    public static final String LEFT_ENCODER_NAME = "FLdrive";
    public static final String RIGHT_ENCODER_NAME = "BRdrive";
    public static final String STRAFE_ENCODER_NAME = "FRdrive";

    // Odometry pod encoder objects
    private final DcMotorEx leftEncoder;
    private final DcMotorEx rightEncoder;
    private final DcMotorEx strafeEncoder;

    // Ticks per inch for converting encoder readings to distances
    public static double TICKS_PER_REV = DriveConstants.ODOMETRY_TICKS_PER_REV;
    public static double WHEEL_RADIUS = DriveConstants.ODOMETRY_WHEEL_RADIUS; // in inches
    public static double GEAR_RATIO = 1; // Assumed direct 1:1 connection

    // Calculated ticks per inch based on wheel circumference
    public static double TICKS_PER_INCH = TICKS_PER_REV / (2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO);

    // Positions of the tracking wheels relative to the center of the robot (in inches)
    public static double LEFT_X = DriveConstants.ODOMETRY_LEFT_X;   // X position of the left wheel
    public static double LEFT_Y = DriveConstants.ODOMETRY_LEFT_Y;   // Y position of the left wheel

    public static double RIGHT_X = DriveConstants.ODOMETRY_RIGHT_X; // X position of the right wheel
    public static double RIGHT_Y = DriveConstants.ODOMETRY_RIGHT_Y; // Y position of the right wheel

    public static double STRAFE_X = DriveConstants.ODOMETRY_MIDDLE_X; // X position of the strafe wheel
    public static double STRAFE_Y = DriveConstants.ODOMETRY_MIDDLE_Y; // Y position of the strafe wheel

    // Wheel encoder directions
    public static boolean REVERSE_LEFT_ENCODER = true;
    public static boolean REVERSE_RIGHT_ENCODER = true;
    public static boolean REVERSE_STRAFE_ENCODER = false;

    // Encoder position tracking
    private int leftEncoderLastPosition = 0;
    private int rightEncoderLastPosition = 0;
    private int strafeEncoderLastPosition = 0;

    /**
     * Constructor for the three-wheel localizer
     * @param hardwareMap hardware mapping from the configuration
     */
    public ThreeWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(LEFT_X, LEFT_Y, 0), // left wheel position
                new Pose2d(RIGHT_X, RIGHT_Y, 0), // right wheel position
                new Pose2d(STRAFE_X, STRAFE_Y, Math.toRadians(90)) // strafe wheel position (90 degrees from forward)
        ));

        // Initialize encoders from hardware map
        leftEncoder = hardwareMap.get(DcMotorEx.class, LEFT_ENCODER_NAME);
        rightEncoder = hardwareMap.get(DcMotorEx.class, RIGHT_ENCODER_NAME);
        strafeEncoder = hardwareMap.get(DcMotorEx.class, STRAFE_ENCODER_NAME);

        // Reset encoder positions
        leftEncoderLastPosition = leftEncoder.getCurrentPosition();
        rightEncoderLastPosition = rightEncoder.getCurrentPosition();
        strafeEncoderLastPosition = strafeEncoder.getCurrentPosition();
    }

    /**
     * Gets the current encoder positions in wheel ticks
     * @return List of current encoder positions for the three wheels
     */
    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPosition = leftEncoder.getCurrentPosition();
        int rightPosition = rightEncoder.getCurrentPosition();
        int strafePosition = strafeEncoder.getCurrentPosition();

        return Arrays.asList(
            encoderTicksToInches(REVERSE_LEFT_ENCODER ? -leftPosition : leftPosition),
            encoderTicksToInches(REVERSE_RIGHT_ENCODER ? -rightPosition : rightPosition),
            encoderTicksToInches(REVERSE_STRAFE_ENCODER ? -strafePosition : strafePosition)
        );
    }

    /**
     * Gets the change in encoder readings since the last call
     * @return List of encoder position deltas for the three wheels
     */
    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // Get current positions
        int leftPosition = leftEncoder.getCurrentPosition();
        int rightPosition = rightEncoder.getCurrentPosition();
        int strafePosition = strafeEncoder.getCurrentPosition();

        // Calculate change in position
        int leftChange = leftPosition - leftEncoderLastPosition;
        int rightChange = rightPosition - rightEncoderLastPosition;
        int strafeChange = strafePosition - strafeEncoderLastPosition;

        // Update last positions
        leftEncoderLastPosition = leftPosition;
        rightEncoderLastPosition = rightPosition;
        strafeEncoderLastPosition = strafePosition;

        // Calculate velocities (change in position)
        return Arrays.asList(
            encoderTicksToInches(REVERSE_LEFT_ENCODER ? -leftChange : leftChange),
            encoderTicksToInches(REVERSE_RIGHT_ENCODER ? -rightChange : rightChange),
            encoderTicksToInches(REVERSE_STRAFE_ENCODER ? -strafeChange : strafeChange)
        );
    }

    /**
     * Converts encoder ticks to inches traveled
     * @param ticks encoder ticks
     * @return inches traveled
     */
    public static double encoderTicksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }
}