/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This OpMode demonstrates using RoadRunner for precise position tracking and autonomous navigation.
 * It implements a sample trajectory that follows a curved path in a triangular shape.
 */
@Autonomous(name="Mecanum Auto with RoadRunner", group="Auto")
public class MecanumAutoWithRoadRunner extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private SampleMecanumDrive drive;
    private Telemetry dashboardTelemetry;
    
    // Starting pose (X, Y, heading in radians)
    private static final Pose2d STARTING_POSE = new Pose2d(0, 0, Math.toRadians(0));
    
    @Override
    public void runOpMode() {
        // Initialize RoadRunner drive
        drive = new SampleMecanumDrive(hardwareMap);
        
        // Set the initial pose estimate
        drive.setCurrentPose(STARTING_POSE);
        
        // Setup dashboard telemetry
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Build trajectories
        buildTrajectories();
        
        // Wait for the game to start (driver presses PLAY)
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.addData("Path", "Ready to run RoadRunner trajectory");
        dashboardTelemetry.update();

        waitForStart();
        runtime.reset();

        // Execute the trajectories
        runTrajectories();
        
        // Report completion
        dashboardTelemetry.addData("Path", "Complete");
        dashboardTelemetry.update();
    }
    
    /**
     * Builds the trajectories for the autonomous path.
     * This is a triangular path similar to the original Pedro pathing example,
     * but implemented with RoadRunner.
     */
    private void buildTrajectories() {
        // Move left 6 feet (72 inches) and forward 2 feet (24 inches) along a curved path
        
        /* 
         * TRAJECTORY EXPLANATION:
         * 
         * This method builds a complex trajectory sequence that moves the robot in a triangular path.
         * The robot will:
         * 1. Start at the origin (0,0) facing forward (0 degrees)
         * 2. Move to position (-72, 24) which is 6 feet left and 2 feet forward
         * 3. Turn to face 90 degrees (facing forward)
         * 4. Move to position (0, 24) which is back to the starting X but 2 feet forward
         * 5. Turn to face 180 degrees (facing backward)
         * 6. Return to the starting position (0, 0)
         * 7. Turn to face the original direction (0 degrees)
         *
         * The trajectory uses splines for smooth curved motion between points, controlled
         * by the velocity and acceleration constraints defined in DriveConstants.
         */
    }
    
    /**
     * Executes the pre-built trajectories.
     * This method shows two different ways to build trajectories:
     * 1. Simple trajectory with line segments
     * 2. Complex trajectory sequence with curved paths and turns
     */
    private void runTrajectories() {
        // APPROACH 1: Simple trajectory with line segments
        dashboardTelemetry.addData("Path", "Starting basic trajectory");
        dashboardTelemetry.update();
        
        // Build a simple trajectory to move in a triangle
        Trajectory trianglePath1 = drive.trajectoryBuilder(STARTING_POSE)
            .lineTo(new Vector2d(-72, 24))  // Move left 6 feet and forward 2 feet
            .build();
            
        Trajectory trianglePath2 = drive.trajectoryBuilder(trianglePath1.end())
            .lineTo(new Vector2d(0, 24))    // Move right 6 feet while staying 2 feet forward
            .build();
            
        Trajectory trianglePath3 = drive.trajectoryBuilder(trianglePath2.end())
            .lineTo(new Vector2d(0, 0))     // Return to start
            .build();
        
        // Execute the segments
        drive.followTrajectory(trianglePath1);
        drive.followTrajectory(trianglePath2);
        drive.followTrajectory(trianglePath3);
        
        sleep(1000);  // Pause between demonstrations
        
        // APPROACH 2: Complex trajectory sequence with curved paths and turns
        dashboardTelemetry.addData("Path", "Starting complex trajectory sequence");
        dashboardTelemetry.update();
        
        // Reset pose estimate to starting position
        drive.setCurrentPose(STARTING_POSE);
        
        // Build a more complex and fluid trajectory sequence
        TrajectorySequence triangleSequence = drive.trajectorySequenceBuilder(STARTING_POSE)
            // First leg: curve to the left side with a spline path
            .splineTo(new Vector2d(-72, 24), Math.toRadians(90))
            // Turn to face forward (90 degrees)
            .turn(Math.toRadians(90))
            // Second leg: curve to the right with a spline path
            .splineTo(new Vector2d(0, 24), Math.toRadians(180))
            // Turn to face backward (180 degrees)
            .turn(Math.toRadians(90))
            // Third leg: return to start with a spline path
            .splineTo(new Vector2d(0, 0), Math.toRadians(270))
            // Turn to face forward again (0 degrees)
            .turn(Math.toRadians(90))
            .build();
            
        // Execute the sequence
        drive.followTrajectorySequence(triangleSequence);
        
        // Display the final position
        Pose2d finalPose = drive.getCurrentPose();
        dashboardTelemetry.addData("Final Position", "X: %.2f, Y: %.2f, Heading: %.2f degrees", 
                              finalPose.getX(), finalPose.getY(), Math.toDegrees(finalPose.getHeading()));
        dashboardTelemetry.update();
    }
}