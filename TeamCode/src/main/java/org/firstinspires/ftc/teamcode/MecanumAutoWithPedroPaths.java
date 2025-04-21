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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.github.pedropedropis.paths.PedroPaths;
import com.github.pedropedropis.paths.PathSet;
import com.github.pedropedropis.paths.Path;
import com.github.pedropedropis.paths.followers.HolonomicPIDFollower;
import com.github.pedropedropis.paths.followers.PathFollower;
import com.github.pedropedropis.paths.motionprofile.TrapezoidalMotionProfile;
import com.github.pedropedropis.paths.geometry.Pose2d;

@Autonomous(name="Mecanum Auto with PedroPaths", group="Auto")
public class MecanumAutoWithPedroPaths extends LinearOpMode {

    // Declare OpMode members for motors
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    
    // Slide motor with encoder
    private DcMotorEx verticalSlide = null;
    
    // Constants for slide positions
    private static final int SLIDE_TARGET_POSITION = 2000;
    private static final int SLIDE_HOME_POSITION = 0;
    
    // PedroPaths variables
    private PathFollower pathFollower;
    private PathSet pathSet;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        verticalSlide = hardwareMap.get(DcMotorEx.class, "vertical_slide");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Configure vertical slide motor
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Initialize PedroPaths
        initPedroPaths();
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Path", "Press play to start autonomous path following");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        // Run the autonomous path
        runAutonomousPath();
    }
    
    private void initPedroPaths() {
        // Create path follower with PID controllers for each motion component
        pathFollower = new HolonomicPIDFollower(
            0.05, 0.0, 0.0, // X PID coefficients
            0.05, 0.0, 0.0, // Y PID coefficients
            0.05, 0.0, 0.0  // Heading PID coefficients
        );
        
        // Create path set
        pathSet = new PathSet();
        
        // Define starting position
        Pose2d startPose = new Pose2d(0, 0, 0); // x, y, heading (in radians)
        
        // Create main path
        Path mainPath = new Path(startPose);
        
        // Add waypoints to the path
        // Move forward 24 inches
        mainPath.lineTo(24, 0);
        
        // Turn 90 degrees right and move forward 24 inches
        mainPath.turnTo(Math.PI/2); // 90 degrees in radians
        mainPath.lineTo(24, 24);
        
        // Turn 90 degrees left and move forward 24 inches
        mainPath.turnTo(0);
        mainPath.lineTo(48, 24);
        
        // Add motion profile to the path
        mainPath.setMotionProfile(new TrapezoidalMotionProfile(
            30.0, // Max velocity in inches per second
            15.0  // Max acceleration in inches per second squared
        ));
        
        // Add path to the set
        pathSet.addPath("main", mainPath);
    }
    
    private void runAutonomousPath() {
        // Get the main path from the set
        Path path = pathSet.getPath("main");
        
        // Set current position to starting position
        Pose2d currentPose = path.getStartPose();
        
        // Start following the path
        pathFollower.followPath(path);
        
        // Loop until path is complete or opmode is stopped
        while (opModeIsActive() && !pathFollower.isFinished()) {
            // Calculate motor powers based on current position and path
            double[] powers = pathFollower.update(currentPose);
            
            // Extract individual motor powers from the array
            double frontLeftPower = powers[0];
            double frontRightPower = powers[1];
            double backLeftPower = powers[2];
            double backRightPower = powers[3];
            
            // Normalize powers if necessary
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }
            
            // Set powers to motors
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            
            // Update current position estimate
            // In a real implementation, this would come from odometry or sensors
            // For this example, we're simulating a simple position update
            currentPose = simulatePositionUpdate(currentPose, powers);
            
            // Display path following info
            telemetry.addData("Status", "Following path: %s", path.getName());
            telemetry.addData("Position", "X: %.2f, Y: %.2f, Heading: %.2f", 
                             currentPose.x, currentPose.y, Math.toDegrees(currentPose.heading));
            telemetry.addData("Progress", "%.2f%%", pathFollower.getProgress() * 100);
            telemetry.update();
        }
        
        // Stop all motors when path is complete
        stopMotors();
        
        // Raise the slide when path is complete
        operateSlide();
        
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    
    private void operateSlide() {
        // Move slide to target position
        telemetry.addData("Slide", "Moving to target position");
        telemetry.update();
        
        verticalSlide.setTargetPosition(SLIDE_TARGET_POSITION);
        verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlide.setPower(0.7);
        
        // Wait until slide reaches position or timeout
        ElapsedTime slideTimer = new ElapsedTime();
        while (opModeIsActive() && 
               verticalSlide.isBusy() && 
               slideTimer.seconds() < 2.0) {
            telemetry.addData("Slide", "Position: %d / %d", 
                             verticalSlide.getCurrentPosition(), 
                             SLIDE_TARGET_POSITION);
            telemetry.update();
            sleep(10);
        }
        
        // Keep holding position
        verticalSlide.setPower(0.1);
    }
    
    private void stopMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    
    /**
     * Simple position simulation - NOT for actual use
     * This is a placeholder for actual odometry feedback
     */
    private Pose2d simulatePositionUpdate(Pose2d currentPose, double[] powers) {
        // This is a very simplified simulation
        // In a real robot, you would use encoders, IMU, or other sensors
        
        double dt = 0.02; // Assume 20ms loop time
        
        // Calculate motion based on powers
        double dx = ((powers[0] + powers[1] + powers[2] + powers[3]) / 4) * Math.cos(currentPose.heading) * dt * 5;
        double dy = ((powers[0] + powers[1] + powers[2] + powers[3]) / 4) * Math.sin(currentPose.heading) * dt * 5;
        double dh = ((powers[0] - powers[1] + powers[2] - powers[3]) / 4) * dt * 2;
        
        // Return updated pose
        return new Pose2d(
            currentPose.x + dx,
            currentPose.y + dy,
            currentPose.heading + dh
        );
    }
}