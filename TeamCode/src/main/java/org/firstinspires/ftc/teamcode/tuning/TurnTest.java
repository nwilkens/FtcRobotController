package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple routine to test turning capabilities.
 * Turn angles can be specified to tune the heading PID controller.
 */
@Config
@Autonomous(group = "tuning")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // in degrees

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry with dashboard support
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the drive system
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Wait for the driver to press start
        telemetry.addData("Status", "Ready to start");
        telemetry.update();
        
        waitForStart();

        if (isStopRequested()) return;

        // Turn the specified angle
        telemetry.addData("Status", "Turning " + ANGLE + " degrees");
        telemetry.update();
        
        // Convert angle to radians and execute the turn
        drive.turn(Math.toRadians(ANGLE));
        
        // Pause briefly
        sleep(1000);
        
        // Turn back to the original heading
        telemetry.addData("Status", "Turning back to start");
        telemetry.update();
        drive.turn(Math.toRadians(-ANGLE));

        // Report the final heading error
        Pose2d finalPose = drive.getCurrentPose();
        telemetry.addData("Final Heading (deg)", Math.toDegrees(finalPose.getHeading()));
        telemetry.addData("Status", "Complete");
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {
            // Keep updating the telemetry
            sleep(50);
        }
    }
}