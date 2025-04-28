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
 * This is a simple routine to test spline paths.
 * Use this op mode to tune your spline following accuracy.
 */
@Config
@Autonomous(group = "tuning")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry with dashboard support
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the drive system
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Start at the origin facing the positive x direction
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setCurrentPose(startPose);

        // Wait for the driver to press start
        telemetry.addData("Status", "Ready to start");
        telemetry.update();
        
        waitForStart();

        if (isStopRequested()) return;

        // Create a spline curve path
        Trajectory trajectory = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(30, 30), 0)
                .build();
                
        // Create a path to return to start
        Trajectory returnTrajectory = drive.trajectoryBuilder(trajectory.end())
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        // Follow the trajectory
        telemetry.addData("Status", "Following spline path");
        telemetry.update();
        drive.followTrajectory(trajectory);
        
        // Pause briefly
        sleep(500);
        
        // Return to start
        telemetry.addData("Status", "Returning to start");
        telemetry.update();
        drive.followTrajectory(returnTrajectory);

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