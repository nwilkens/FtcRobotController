package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple routine to test translational drive capabilities.
 * It drives forward for a specified distance, then backward to the starting position.
 */
@Config
@Autonomous(group = "tuning")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in inches

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry with dashboard support
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the drive system
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Create a trajectory to move forward
        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        // Create a trajectory to move back to start
        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .back(DISTANCE)
                .build();

        // Wait for the driver to press start
        telemetry.addData("Status", "Ready to start");
        telemetry.update();
        
        waitForStart();

        if (isStopRequested()) return;

        // Drive the specified distance forward
        telemetry.addData("Status", "Driving forward");
        telemetry.update();
        drive.followTrajectory(trajectoryForward);
        
        // Pause briefly
        sleep(500);
        
        // Drive back to the starting position
        telemetry.addData("Status", "Driving backward");
        telemetry.update();
        drive.followTrajectory(trajectoryBackward);

        // Report the final position error
        Pose2d finalPose = drive.getCurrentPose();
        telemetry.addData("Final X", finalPose.getX());
        telemetry.addData("Final Y", finalPose.getY());
        telemetry.addData("Final Heading", Math.toDegrees(finalPose.getHeading()));
        telemetry.addData("Status", "Complete");
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {
            // Keep updating the telemetry
            sleep(50);
        }
    }
}