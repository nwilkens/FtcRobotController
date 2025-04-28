package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This is the RoadRunner Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 * This is a direct replacement for the Pedro-based Triangle class.
 */
@Autonomous(name = "RoadRunner Triangle", group = "Examples")
public class RoadRunnerTriangle extends LinearOpMode {
    // The drive system that will run the trajectory
    private SampleMecanumDrive drive;
    
    // Dashboard telemetry for visualization
    private Telemetry dashboardTelemetry;
    
    // The three points of the triangle (in field coordinates)
    private final Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    private final Pose2d interPose = new Pose2d(24, -24, Math.toRadians(90));
    private final Pose2d endPose = new Pose2d(24, 24, Math.toRadians(45));

    @Override
    public void runOpMode() {
        // Initialize the drive system
        drive = new SampleMecanumDrive(hardwareMap);
        
        // Set the initial pose
        drive.setCurrentPose(startPose);
        
        // Initialize dashboard telemetry
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Build the triangle trajectory sequence
        TrajectorySequence triangleSequence = buildTriangleSequence();
        
        // Display initialization message
        dashboardTelemetry.addLine("This will run in a roughly triangular shape,"
                + "starting on the bottom-middle point. So, make sure you have enough "
                + "space to the left, front, and right to run the OpMode.");
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
        
        // Wait for the start button
        waitForStart();
        
        // Execute the trajectory
        drive.followTrajectorySequence(triangleSequence);
        
        // Report completion
        dashboardTelemetry.addData("Path", "Complete");
        dashboardTelemetry.addData("Final Position", "X: %.2f, Y: %.2f, Heading: %.2f°", 
                              drive.getCurrentPose().getX(), 
                              drive.getCurrentPose().getY(), 
                              Math.toDegrees(drive.getCurrentPose().getHeading()));
        dashboardTelemetry.update();
    }
    
    /**
     * Builds the triangle trajectory sequence.
     * 
     * This method creates a complete triangle path using RoadRunner's trajectory sequence builder.
     * The path consists of three segments:
     * 1. Start position to intermediate position (bottom-left corner)
     * 2. Intermediate position to end position (top corner)
     * 3. End position back to start position (bottom-middle corner)
     * 
     * Each segment uses spline paths for smooth motion and includes heading interpolation
     * to correctly orient the robot throughout the motion.
     * 
     * @return The complete triangle trajectory sequence
     */
    private TrajectorySequence buildTriangleSequence() {
        /*
         * TRAJECTORY EXPLANATION:
         * 
         * We're creating a triangle path with three spline segments:
         * 
         * 1. First segment: From start (0,0) to intermediate point (24,-24)
         *    - Uses splineToLinearHeading to smoothly curve to the position
         *    - Rotates from 0° to 90° heading during the motion
         * 
         * 2. Second segment: From intermediate (24,-24) to end point (24,24)
         *    - Uses splineToLinearHeading to smoothly curve to the position
         *    - Rotates from 90° to 45° heading during the motion
         * 
         * 3. Third segment: From end (24,24) back to start (0,0)
         *    - Uses splineToLinearHeading to smoothly curve to the position
         *    - Rotates from 45° back to 0° heading during the motion
         * 
         * The spline paths create a smooth curved motion between points,
         * unlike simple line segments which would create sharp corners.
         */
        return drive.trajectorySequenceBuilder(startPose)
                // First segment: start to intermediate
                .splineToLinearHeading(interPose, Math.toRadians(0))
                
                // Second segment: intermediate to end
                .splineToLinearHeading(endPose, Math.toRadians(90))
                
                // Third segment: end back to start
                .splineToLinearHeading(startPose, Math.toRadians(225))
                
                .build();
    }
    
    /**
     * Alternative implementation using individual turn commands.
     * This shows how you could build the same triangle using discrete segments
     * and turn commands if you wanted more control over each part of the motion.
     */
    private TrajectorySequence buildAlternativeTriangleSequence() {
        return drive.trajectorySequenceBuilder(startPose)
                // Move to first corner
                .lineTo(new Vector2d(24, -24))
                
                // Turn to face 90 degrees
                .turn(Math.toRadians(90))
                
                // Move to second corner
                .lineTo(new Vector2d(24, 24))
                
                // Turn to face 45 degrees
                .turn(Math.toRadians(-45))
                
                // Return to start
                .lineTo(new Vector2d(0, 0))
                
                // Turn to face 0 degrees
                .turn(Math.toRadians(-45))
                
                .build();
    }
}