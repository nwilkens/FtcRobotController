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

@TeleOp(name="Mecanum Drive", group="TeleOp")
public class MecanumDriveTeleOp extends LinearOpMode {

    // Declare OpMode members for motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    
    // Slide motor with encoder
    private DcMotorEx verticalSlide = null;
    
    // Constants for slide positions
    private static final int SLIDE_TARGET_POSITION = 2000;
    private static final int SLIDE_HOME_POSITION = 0;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        verticalSlide = hardwareMap.get(DcMotorEx.class, "vertical_slide");

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward 
        // and observe the direction the wheels turn.
        // Reverse the direction of motors that run backward when you push forward.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Configure vertical slide motor for position control
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlide.setDirection(DcMotorSimple.Direction.FORWARD); // Adjust if needed
        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Get gamepad inputs
            double drive = -gamepad1.left_stick_y;  // Forward/backward
            double strafe = gamepad1.left_stick_x;  // Left/right
            double turn = gamepad1.right_stick_x;   // Rotation

            // Calculate motor powers using mecanum drive kinematics
            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
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
            telemetry.addData("Front Motors", "left: %.2f, right: %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back Motors", "left: %.2f, right: %.2f", backLeftPower, backRightPower);
            telemetry.addData("Vertical Slide", "Position: %d, Target: %d", 
                              verticalSlide.getCurrentPosition(), 
                              verticalSlide.getTargetPosition());
            telemetry.update();
        }
    }
}