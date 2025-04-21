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

import com.acmerobotics.roadrunner.control.PIDCoefficients;
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

import java.util.Arrays;

@Autonomous(name="Mecanum Auto with RoadRunner", group="Auto")
public class MecanumAutoWithRoadRunner extends LinearOpMode {

    // Declare OpMode members for motors
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    
    // Slide motor with encoder
    private DcMotorEx verticalSlide = null;
    
    // Constants for slide positions
    private static final int SLIDE_TARGET_POSITION = 2000;
    private static final int SLIDE_HOME_POSITION = 0;
    
    // RoadRunner variables
    private RoadRunnerMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();
    
    // Constants for RoadRunner
    public static final double TICKS_PER_REV = 537.7; // GoBILDA 5202 Series 19.2:1
    public static final double MAX_RPM = 312;
    public static final double WHEEL_RADIUS = 2; // in
    public static final double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static final double TRACK_WIDTH = 14; // in
    public static final double WHEEL_BASE = 14; // in

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "back_right");
        verticalSlide = hardwareMap.get(DcMotorEx.class, "vertical_slide");

        // Configure motors
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Configure vertical slide motor
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Initialize RoadRunner drive
        drive = new RoadRunnerMecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Path", "Press play to start autonomous path following");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        // Run the autonomous trajectory
        runAutonomousPath();
    }
    
    private void runAutonomousPath() {
        // Create trajectory
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
            .forward(24) // Move forward 24 inches
            .strafeRight(24) // Strafe right 24 inches
            .forward(24) // Move forward another 24 inches
            .build();
        
        // Follow trajectory
        telemetry.addData("Path", "Starting trajectory");
        telemetry.update();
        drive.followTrajectory(trajectory);
        
        // Stop all motors when trajectory is complete
        drive.setDrivePower(new Pose2d());
        
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
    
    /**
     * RoadRunner Mecanum Drive implementation 
     */
    private class RoadRunnerMecanumDrive extends MecanumDrive {
        private DcMotorEx leftFront, rightFront, leftRear, rightRear;
        private TrajectoryFollower follower;
        
        public RoadRunnerMecanumDrive(DcMotorEx leftFront, DcMotorEx rightFront, 
                                     DcMotorEx leftRear, DcMotorEx rightRear) {
            super(TRACK_WIDTH, WHEEL_BASE, WHEEL_RADIUS, kV, kA, kStatic);
            
            this.leftFront = leftFront;
            this.rightFront = rightFront;
            this.leftRear = leftRear;
            this.rightRear = rightRear;
            
            // Zero encoders
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            // Set run mode
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            // Zero behavior
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            // Initialize follower
            follower = new HolonomicPIDVAFollower(
                new PIDCoefficients(5, 0, 0), // translational PID
                new PIDCoefficients(5, 0, 0), // heading PID
                0.5, 0.5, 10.0 // Timeout, max corrections per sec, max correction velocity
            );
        }
        
        public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
            return new TrajectoryBuilder(startPose, getVelocityConstraint(), getAccelerationConstraint());
        }
        
        public void followTrajectory(Trajectory trajectory) {
            follower.followTrajectory(trajectory);
            
            ElapsedTime timer = new ElapsedTime();
            Pose2d currentPose = getPoseEstimate();
            
            while (opModeIsActive() && !follower.isFinished()) {
                // Update pose estimate
                currentPose = updatePoseEstimate();
                
                // Calculate motor powers
                Pose2d correction = follower.update(currentPose);
                Pose2d targetVelocity = follower.getLastError();
                
                // Calculate and set wheel powers
                setDriveSignal(getMecanumSignal(targetVelocity, correction));
                
                // Display path following info
                telemetry.addData("Position", "X: %.2f, Y: %.2f, Heading: %.2f", 
                                currentPose.getX(), currentPose.getY(), 
                                Math.toDegrees(currentPose.getHeading()));
                telemetry.addData("Time", "%.2f seconds", timer.seconds());
                telemetry.update();
            }
            
            // Stop motors
            setDrivePower(new Pose2d());
        }
        
        public void setDrivePower(Pose2d drivePower) {
            // Set individual motor powers using mecanum kinematics
            double leftFrontPower = drivePower.getX() - drivePower.getY() - drivePower.getHeading();
            double rightFrontPower = drivePower.getX() + drivePower.getY() + drivePower.getHeading();
            double leftRearPower = drivePower.getX() + drivePower.getY() - drivePower.getHeading();
            double rightRearPower = drivePower.getX() - drivePower.getY() + drivePower.getHeading();
            
            // Normalize powers if any exceed 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftRearPower));
            max = Math.max(max, Math.abs(rightRearPower));
            
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftRearPower /= max;
                rightRearPower /= max;
            }
            
            // Set powers
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftRear.setPower(leftRearPower);
            rightRear.setPower(rightRearPower);
        }
        
        public Pose2d getPoseEstimate() {
            // In a real implementation, you would use proper odometry or sensors
            // For this example, we're using a simulated position estimate
            double leftFrontPos = encoderTicksToInches(leftFront.getCurrentPosition());
            double rightFrontPos = encoderTicksToInches(rightFront.getCurrentPosition());
            double leftRearPos = encoderTicksToInches(leftRear.getCurrentPosition());
            double rightRearPos = encoderTicksToInches(rightRear.getCurrentPosition());
            
            // Simple odometry calculation (not accurate for real use)
            double x = (leftFrontPos + rightFrontPos + leftRearPos + rightRearPos) / 4.0;
            double y = (rightFrontPos + leftRearPos - leftFrontPos - rightRearPos) / 4.0;
            double heading = (rightFrontPos + rightRearPos - leftFrontPos - leftRearPos) / (4.0 * TRACK_WIDTH);
            
            return new Pose2d(x, y, heading);
        }
        
        public Pose2d updatePoseEstimate() {
            // Update the pose estimate (in a real implementation, this would use sensors)
            return getPoseEstimate();
        }
        
        private double encoderTicksToInches(int ticks) {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
        }
        
        private TrajectoryVelocityConstraint getVelocityConstraint() {
            return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
            ));
        }
        
        private TrajectoryAccelerationConstraint getAccelerationConstraint() {
            return new ProfileAccelerationConstraint(MAX_ACCEL);
        }
        
        // Constants for the drive
        private static final double MAX_VEL = 30;           // Max linear velocity in in/s
        private static final double MAX_ACCEL = 15;         // Max linear acceleration in in/s^2  
        private static final double MAX_ANG_VEL = Math.PI;  // Max angular velocity in rad/s
        private static final double MAX_ANG_ACCEL = Math.PI/2; // Max angular acceleration in rad/s^2
        
        // Motor feedforward constants
        private static final double kV = 1.0 / (MAX_RPM / 60 * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS); // V/(in/s)
        private static final double kA = 0;                 // V/(in/s^2)
        private static final double kStatic = 0.1;          // V
    }
}