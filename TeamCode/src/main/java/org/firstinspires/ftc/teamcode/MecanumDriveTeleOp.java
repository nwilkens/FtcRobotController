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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Mecanum Drive2", group="TeleOp")
public class MecanumDriveTeleOp extends LinearOpMode {

    // Declare OpMode members for motors and sensors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private SparkFunOTOS odometrySensor = null;
    

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "FLdrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FRdrive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "BLdrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BRdrive");

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward 
        // and observe the direction the wheels turn.
        // Reverse the direction of motors that run backward when you push forward.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Set all motors to BRAKE mode when power is zero
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize OTOS sensor (make sure this matches your configuration name)
        try {
            odometrySensor = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
            configureOtosSensor();
            telemetry.addData("OTOS Sensor", "Initialized");
        } catch (Exception e) {
            telemetry.addData("OTOS Sensor", "Not found! Error: %s", e.getMessage());
            odometrySensor = null;
        }

        // Setup dashboard telemetry
        telemetry = DashboardInitialization.createTelemetry(telemetry);
        
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
            
            // Apply speed scaling if left bumper is pressed
            double speedMultiplier = gamepad1.left_bumper ? 0.5 : 1.0;

            // Calculate motor powers using mecanum drive kinematics
            double frontLeftPower = (drive + strafe + turn) * speedMultiplier;
            double frontRightPower = (drive - strafe - turn) * speedMultiplier;
            double backLeftPower = (drive - strafe + turn) * speedMultiplier;
            double backRightPower = (drive + strafe - turn) * speedMultiplier;

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


            // Show the elapsed game time and motor information
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed Mode", gamepad1.left_bumper ? "50%" : "100%");
            telemetry.addData("Front Motors", "left: %.2f, right: %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back Motors", "left: %.2f, right: %.2f", backLeftPower, backRightPower);
            
            // Add OTOS position data to telemetry
            if (odometrySensor != null) {
                // Check for reset request
                if (gamepad1.triangle) {
                    odometrySensor.resetTracking();
                    telemetry.addData("OTOS", "Position Reset!");
                }
                
                // Check for IMU calibration request
                if (gamepad1.square) {
                    odometrySensor.calibrateImu();
                    telemetry.addData("OTOS", "IMU Calibrated!");
                }
                
                // Get and display position data
                SparkFunOTOS.Pose2D position = odometrySensor.getPosition();
                telemetry.addData("Position", "X: %.2f, Y: %.2f", position.x, position.y);
                telemetry.addData("Heading", "%.2f", position.h);
                telemetry.addLine("Press TRIA to reset position tracking");
                telemetry.addLine("Press SQUA to calibrate IMU");
            }
            
            telemetry.update();
        }
    }
    
    /**
     * Configure the OTOS sensor with initial settings
     */
    private void configureOtosSensor() {
        // Set measurement units (inches and degrees)
        odometrySensor.setLinearUnit(DistanceUnit.INCH);
        odometrySensor.setAngularUnit(AngleUnit.DEGREES);
        
        // Set sensor offset relative to robot center if needed
        // This depends on where you mount the sensor on your robot
        // Example: If mounted 3 inches left and 2 inches forward of center
        // SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3, 2, 0);
        // odometrySensor.setOffset(offset);
        
        // Use a default offset of 0,0,0 for now (adjust based on your mounting)
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        odometrySensor.setOffset(offset);
        
        // Set scaling factors (default 1.0, adjust after calibration)
        odometrySensor.setLinearScalar(1.0);
        odometrySensor.setAngularScalar(1.0);
        
        // Calibrate the IMU
        odometrySensor.calibrateImu();
        
        // Reset tracking
        odometrySensor.resetTracking();
    }
}