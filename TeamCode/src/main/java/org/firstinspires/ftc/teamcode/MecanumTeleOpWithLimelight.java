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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name="Mecanum Drive with Limelight", group="TeleOp")
public class MecanumTeleOpWithLimelight extends LinearOpMode {

    // Declare OpMode members for motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    
    // Slide motor with encoder
    private DcMotorEx verticalSlide = null;
    
    // Limelight vision sensor
    private Limelight3A limelight = null;
    
    // Constants for slide positions
    private static final int SLIDE_TARGET_POSITION = 2000;
    private static final int SLIDE_HOME_POSITION = 0;
    
    // Constants for Limelight tracking
    private static final double TRACKING_SPEED = 0.3;  // Base speed for tracking movements
    private static final double X_DEADBAND = 1.0;      // Deadband for X alignment (degrees)
    private static final double Y_DEADBAND = 1.0;      // Deadband for Y alignment (degrees)
    
    // PID coefficients for smoother motion when tracking
    private static final double P_COEFF = 0.03;        // Proportional coefficient
    
    // Tracking state
    private boolean trackingEnabled = false;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        verticalSlide = hardwareMap.get(DcMotorEx.class, "vertical_slide");
        
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Configure vertical slide motor for position control
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Configure Limelight
        limelight.pipelineSwitch(0);  // Use pipeline 0
        limelight.start();            // Start polling for data

        // Wait for the game to start
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match
        while (opModeIsActive()) {
            // Toggle tracking mode with B button
            if (gamepad1.b && !trackingEnabled) {
                trackingEnabled = true;
                telemetry.addData("Tracking", "ENABLED");
            } else if (gamepad1.x && trackingEnabled) {
                trackingEnabled = false;
                telemetry.addData("Tracking", "DISABLED");
            }
            
            // Calculate drive values
            double drive = 0;
            double strafe = 0;
            double turn = 0;
            
            // Use Limelight tracking if enabled, otherwise use manual control
            if (trackingEnabled) {
                LLResult result = limelight.getLatestResult();
                
                if (result != null && result.isValid()) {
                    // Get target coordinates (tx and ty are in degrees from center)
                    double tx = result.getTx();  // Positive means target is to the right
                    double ty = result.getTy();  // Positive means target is above center
                    
                    // Display target info
                    telemetry.addData("Target", "X: %.2f°, Y: %.2f°", tx, ty);
                    
                    // Only adjust if target is outside deadband
                    if (Math.abs(tx) > X_DEADBAND) {
                        // Apply proportional control for smoother movement
                        turn = -tx * P_COEFF;  // Negative because positive tx means turn right
                        
                        // Limit maximum turn rate
                        turn = Math.max(-TRACKING_SPEED, Math.min(TRACKING_SPEED, turn));
                    }
                    
                    if (Math.abs(ty) > Y_DEADBAND) {
                        // Move forward/backward to adjust y position
                        drive = -ty * P_COEFF;  // Negative because positive ty means drive backward
                        
                        // Limit maximum drive rate
                        drive = Math.max(-TRACKING_SPEED, Math.min(TRACKING_SPEED, drive));
                    }
                    
                    // Allow manual strafe while tracking
                    strafe = gamepad1.left_stick_x * 0.5;  // Reduced sensitivity during tracking
                    
                } else {
                    telemetry.addData("Limelight", "No valid target");
                    trackingEnabled = false;  // Disable tracking if target lost
                }
            } else {
                // Manual control when not tracking
                drive = -gamepad1.left_stick_y;  // Forward/backward
                strafe = gamepad1.left_stick_x;  // Left/right
                turn = gamepad1.right_stick_x;   // Rotation
            }

            // Calculate motor powers using mecanum drive kinematics
            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            // Normalize the values so no wheel power exceeds 100%
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Handle the vertical slide motor
            if (gamepad2.y) {
                // Move to target position when Y is pressed
                verticalSlide.setTargetPosition(SLIDE_TARGET_POSITION);
                verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalSlide.setPower(0.7); // Power level for moving to position
                telemetry.addData("Slide", "Moving to position: %d", SLIDE_TARGET_POSITION);
            } else if (gamepad2.a) {
                // Return to home position when A is pressed
                verticalSlide.setTargetPosition(SLIDE_HOME_POSITION);
                verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalSlide.setPower(0.7); // Power level for moving to position
                telemetry.addData("Slide", "Moving to home position");
            }
            
            // Manual slide control using the right stick on gamepad2 when not in position mode
            if (verticalSlide.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                double slideControl = -gamepad2.right_stick_y;
                verticalSlide.setPower(slideControl * 0.7);
            }
            
            // If slide is done moving to position, switch back to encoder mode
            if (verticalSlide.getMode() == DcMotor.RunMode.RUN_TO_POSITION && 
                !verticalSlide.isBusy()) {
                verticalSlide.setPower(0);
                verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Show the elapsed game time and motor information
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Tracking", trackingEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Motors", "FL:%.2f, FR:%.2f, BL:%.2f, BR:%.2f", 
                             frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Vertical Slide", "Pos: %d", verticalSlide.getCurrentPosition());
            
            // Display Limelight info only when not tracking
            if (!trackingEnabled) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    telemetry.addData("Limelight", "tx: %.2f, ty: %.2f", 
                                    result.getTx(), result.getTy());
                }
            }
            
            telemetry.update();
        }
        
        // Stop Limelight when OpMode ends
        limelight.stop();
    }
}