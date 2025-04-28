package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This is a mecanum drive implementation designed for RoadRunner path following.
 * It uses three-wheel odometry for position tracking.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    
    @Override
    public void setMotorPowers(double frontLeft, double backLeft, double backRight, double frontRight) {
        leftFront.setPower(frontLeft);
        leftRear.setPower(backLeft);
        rightRear.setPower(backRight);
        rightFront.setPower(frontRight);
    }
    
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(encoderTicksToInches(leftFront.getCurrentPosition()));
        wheelPositions.add(encoderTicksToInches(leftRear.getCurrentPosition()));
        wheelPositions.add(encoderTicksToInches(rightRear.getCurrentPosition()));
        wheelPositions.add(encoderTicksToInches(rightFront.getCurrentPosition()));
        return wheelPositions;
    }
    
    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        wheelVelocities.add(encoderTicksToInches(leftFront.getVelocity()));
        wheelVelocities.add(encoderTicksToInches(leftRear.getVelocity()));
        wheelVelocities.add(encoderTicksToInches(rightRear.getVelocity()));
        wheelVelocities.add(encoderTicksToInches(rightFront.getVelocity()));
        return wheelVelocities;
    }
    
    public static double encoderTicksToInches(double ticks) {
        return DriveConstants.WHEEL_RADIUS * 2 * Math.PI * DriveConstants.GEAR_RATIO * ticks / DriveConstants.TICKS_PER_REV;
    }
    // Hardware and tracking variables
    private final TrajectorySequenceRunner trajectorySequenceRunner;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(
            DriveConstants.MAX_VEL, 
            DriveConstants.MAX_ANG_VEL, 
            DriveConstants.TRACK_WIDTH
    );
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(DriveConstants.MAX_ACCEL);
    
    // IMU for heading
    private BNO055IMU imu;

    // Hardware names for the drive motors
    public static final String FRONT_LEFT_MOTOR_NAME = "FLdrive";
    public static final String FRONT_RIGHT_MOTOR_NAME = "FRdrive";
    public static final String BACK_LEFT_MOTOR_NAME = "BLdrive";
    public static final String BACK_RIGHT_MOTOR_NAME = "BRdrive";
    
    // The motors powering the drivetrain
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    
    // Voltage sensor for battery compensation
    private VoltageSensor batteryVoltageSensor;

    /**
     * Our trajectory follower that will accurately track the trajectories we build
     */
    private final TrajectoryFollower follower;

    /**
     * Constructor for the mecanum drive
     * @param hardwareMap hardware mapping from the configuration
     */
    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(DriveConstants.LATERAL_MULTIPLIER, DriveConstants.TRACK_WIDTH, DriveConstants.TRACK_WIDTH, DriveConstants.WHEEL_BASE);

        // Initialize followers with PID coefficients for position tracking
        follower = new HolonomicPIDVAFollower(
                new PIDCoefficients(DriveConstants.TRANSLATIONAL_PID_COEFF_P, 
                                   DriveConstants.TRANSLATIONAL_PID_COEFF_I, 
                                   DriveConstants.TRANSLATIONAL_PID_COEFF_D),
                new PIDCoefficients(DriveConstants.TRANSLATIONAL_PID_COEFF_P, 
                                   DriveConstants.TRANSLATIONAL_PID_COEFF_I, 
                                   DriveConstants.TRANSLATIONAL_PID_COEFF_D),
                new PIDCoefficients(DriveConstants.HEADING_PID_COEFF_P, 
                                   DriveConstants.HEADING_PID_COEFF_I, 
                                   DriveConstants.HEADING_PID_COEFF_D),
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), // Error tolerances
                0.5 // Timeout for waiting to reach position (seconds)
        );

        // Initialize trajectory sequence runner with initial pose
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, new Pose2d());

        // Set up bulk reads for reduced CAN bus traffic
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Get voltage sensor for battery compensation
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        
        // Initialize IMU for heading tracking
        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        } catch (Exception e) {
            // If IMU is not available, we'll use odometry for heading
            imu = null;
        }

        // Initialize drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR_NAME);
        leftRear = hardwareMap.get(DcMotorEx.class, BACK_LEFT_MOTOR_NAME);
        rightRear = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_MOTOR_NAME);
        rightFront = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR_NAME);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        // Configure motor directions based on your robot's configuration
        leftFront.setDirection(DriveConstants.REVERSED_FRONT_LEFT ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DriveConstants.REVERSED_BACK_LEFT ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DriveConstants.REVERSED_FRONT_RIGHT ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DriveConstants.REVERSED_BACK_RIGHT ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // Configure motor parameters
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        // Set appropriate mode for motors
        if (DriveConstants.RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set zero power behavior to BRAKE (stops immediately)
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set PID coefficients for velocity control (if using encoders)
        if (DriveConstants.RUN_USING_ENCODER && DriveConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID);
        }
    }

    /**
     * Sets the drive mode for all motors.
     * @param runMode desired run mode (RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, etc.)
     */
    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    /**
     * Sets the zero power behavior for all drive motors.
     * @param zeroPowerBehavior behavior when motors have zero power (BRAKE or FLOAT)
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    /**
     * Sets PID coefficients for the drive motors.
     * @param runMode whether the motors are running using encoders or not
     * @param coefficients the PID coefficients to use
     */
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    /**
     * Creates a trajectory builder for this drive.
     * @param startPose the starting pose for the trajectory
     * @return a trajectory builder instance
     */
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    /**
     * Creates a trajectory builder with a custom starting tangent.
     * @param startPose the starting pose for the trajectory
     * @param startHeading the tangent angle for the start of the trajectory
     * @return a trajectory builder instance
     */
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    /**
     * Creates a trajectory sequence builder for chaining multiple movements.
     * @param startPose the starting pose for the trajectory sequence
     * @return a trajectory sequence builder instance
     */
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        );
    }

    /**
     * Follows a trajectory.
     * @param trajectory the trajectory to follow
     */
    public void followTrajectory(Trajectory trajectory) {
        // Create a simple sequence with just this trajectory
        TrajectorySequence trajectorySequence = trajectorySequenceBuilder(trajectory.start())
                .lineTo(trajectory.end().vec())
                .build();
                
        trajectorySequenceRunner.followTrajectorySequence(trajectorySequence);
    }

    /**
     * Follows a trajectory sequence.
     * @param trajectorySequence the trajectory sequence to follow
     */
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequence(trajectorySequence);
    }

    // Since the parent class has final methods for pose estimation,
    // we need to use different method names.
    
    /**
     * Returns the current pose estimate of the robot.
     * @return the current pose estimate
     */
    public Pose2d getCurrentPose() {
        // Use the parent's getPoseEstimate() method
        return getPoseEstimate();
    }

    /**
     * Sets the pose estimate of the robot.
     * @param pose the pose to set
     */
    public void setCurrentPose(Pose2d pose) {
        // Update the pose in the trajectory sequence runner
        trajectorySequenceRunner.setCurrentPose(pose);
        
        // Force an update of the pose
        updatePoseEstimate();
    }
    
    /**
     * Gets the raw heading from the IMU
     * @return raw heading in radians
     */
    @Override
    public double getRawExternalHeading() {
        // Return IMU heading if available
        if (imu != null) {
            return imu.getAngularOrientation().firstAngle;
        }
        
        // Return 0 if no IMU is available (we'll rely on odometry for heading)
        return 0.0;
    }
    
    /**
     * Gets the heading velocity from external sensors
     * @return heading velocity in radians/second or null if not available
     */
    @Override
    public Double getExternalHeadingVelocity() {
        // If IMU is available and provides angular velocity, return it
        if (imu != null) {
            return (double) imu.getAngularVelocity().zRotationRate;
        }
        
        // Return null if not available
        return null;
    }

    /**
     * Updates the robot's position estimate and follows active trajectories.
     * Call this method in the OpMode's loop() method.
     */
    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    /**
     * Turns the robot to face the specified angle (in radians).
     * @param angle the angle to turn to (in radians)
     */
    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    /**
     * Turns the robot to face the specified angle, blocking until completed.
     * @param angle the angle to turn to (in radians)
     */
    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    /**
     * Checks if the drive is currently following a trajectory.
     * @return true if the drive is busy
     */
    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    /**
     * Waits for the drive to finish all trajectory following operations.
     */
    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    /**
     * Sets drive powers using mecanum kinematics.
     * @param drivePowers the powers to set (x, y, and heading components)
     */
    public void setDrivePower(Pose2d drivePowers) {
        double leftFrontPower = drivePowers.getY() + drivePowers.getX() + drivePowers.getHeading();
        double leftRearPower = drivePowers.getY() - drivePowers.getX() + drivePowers.getHeading();
        double rightFrontPower = drivePowers.getY() - drivePowers.getX() - drivePowers.getHeading();
        double rightRearPower = drivePowers.getY() + drivePowers.getX() - drivePowers.getHeading();

        // Normalize wheel powers to ensure no value exceeds 1.0
        double maxPower = Math.max(
            Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower)), 
            Math.max(Math.abs(rightFrontPower), Math.abs(rightRearPower))
        );

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            leftRearPower /= maxPower;
            rightFrontPower /= maxPower;
            rightRearPower /= maxPower;
        }

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }
    
    /**
     * Sets drive powers with input scaling for smoother control.
     * Used primarily for manual driver control.
     * @param drivePowers the powers to set (x, y, and heading components)
     */
    public void setWeightedDrivePower(Pose2d drivePowers) {
        Pose2d scaledPowers;
        
        // Scale the input power for better control
        // Square the inputs (while preserving the sign) to increase precision at low speeds
        double x = Math.signum(drivePowers.getX()) * Math.pow(drivePowers.getX(), 2);
        double y = Math.signum(drivePowers.getY()) * Math.pow(drivePowers.getY(), 2);
        double heading = Math.signum(drivePowers.getHeading()) * Math.pow(drivePowers.getHeading(), 2);
        
        scaledPowers = new Pose2d(x, y, heading);
        
        // Use the setDrivePower method for consistent power application
        setDrivePower(scaledPowers);
    }

    /**
     * Sets drive signal for all motors (used by the trajectory follower).
     * @param driveSignal the drive signal containing target velocities
     */
    @Override
    public void setDriveSignal(DriveSignal driveSignal) {
        setDrivePower(new Pose2d(
                driveSignal.getVel().getX(),
                driveSignal.getVel().getY(),
                driveSignal.getVel().getHeading()
        ));
    }

    /**
     * Get the wheel velocity constraint for trajectory planning.
     * @param maxWheelVel maximum wheel velocity
     * @param maxAngularVel maximum angular velocity
     * @param trackWidth width between wheels
     * @return a velocity constraint
     */
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxWheelVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new MecanumVelocityConstraint(maxWheelVel, trackWidth),
                new AngularVelocityConstraint(maxAngularVel)
        ));
    }

    /**
     * Get acceleration constraint for trajectory planning.
     * @param maxAccel maximum acceleration
     * @return an acceleration constraint
     */
    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}