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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This OpMode demonstrates using the SparkFun OTOS sensor for precise position tracking
 * and implements a simple "Pedro Pathing" to move forward 12 inches and back to start.
 */
@Autonomous(name="Mecanum Auto with OTOS", group="Auto")
public class MecanumAutoWithOTOS extends LinearOpMode {

    // Declare OpMode members for motors and sensors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private SparkFunOTOS odometrySensor = null;
    
    // Constants for PID control
    private static final double P_DRIVE_COEFF = 0.03;  // Proportional coefficient for driving
    private static final double HEADING_THRESHOLD = 1.0;  // Degrees - how close to target heading before moving on
    private static final double POSITION_THRESHOLD = 0.5;  // Inches - how close to target position before moving on
    private static final double MAX_DRIVE_POWER = 0.5;  // Maximum driving power (0.0 to 1.0)

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "FLdrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FRdrive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "BLdrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BRdrive");

        // Set motor directions (same as in TeleOp)
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
        telemetry.addData("Path", "Ready to run Pedro Pathing: 12 inches forward and back");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Make sure OTOS sensor is ready before starting
        if (odometrySensor == null) {
            telemetry.addData("Error", "OTOS sensor not found! Cannot run autonomous.");
            telemetry.update();
            sleep(2000);
            return;
        }

        // Reset position tracking at the start
        odometrySensor.resetTracking();
        sleep(100);  // Small delay to ensure reset completes

        // Get initial position
        SparkFunOTOS.Pose2D startPosition = odometrySensor.getPosition();
        
        telemetry.addData("Starting Position", "X: %.2f, Y: %.2f, H: %.2f", 
                          startPosition.x, startPosition.y, startPosition.h);
        telemetry.update();

        // Pedro Pathing: Move forward 12 inches
        driveToPosition(startPosition.x, startPosition.y + 12.0, startPosition.h, 5.0);
        
        // Small delay
        sleep(500);
        
        // Get current position after first movement
        SparkFunOTOS.Pose2D midPosition = odometrySensor.getPosition();
        telemetry.addData("Mid Position", "X: %.2f, Y: %.2f, H: %.2f", 
                         midPosition.x, midPosition.y, midPosition.h);
        telemetry.update();
        
        // Pedro Pathing: Move back to start position
        driveToPosition(startPosition.x, startPosition.y, startPosition.h, 5.0);

        // Final position report
        SparkFunOTOS.Pose2D endPosition = odometrySensor.getPosition();
        telemetry.addData("End Position", "X: %.2f, Y: %.2f, H: %.2f", 
                          endPosition.x, endPosition.y, endPosition.h);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        
        // Stop all motion
        setAllMotorPowers(0, 0, 0);
        
        sleep(1000);  // Short pause at the end
    }
    
    /**
     * Configure the OTOS sensor with initial settings
     */
    private void configureOtosSensor() {
        // Set measurement units (inches and degrees)
        odometrySensor.setLinearUnit(DistanceUnit.INCH);
        odometrySensor.setAngularUnit(AngleUnit.DEGREES);
        
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
    
    /**
     * Pedro Pathing implementation to drive to a specific position with heading control
     */
    public void driveToPosition(double targetX, double targetY, double targetHeading, double timeoutS) {
        ElapsedTime timer = new ElapsedTime();
        double distanceError;
        double headingError;
        
        // Initialize errors
        SparkFunOTOS.Pose2D currentPosition = odometrySensor.getPosition();
        distanceError = Math.hypot(targetX - currentPosition.x, targetY - currentPosition.y);
        headingError = targetHeading - currentPosition.h;
        
        // Normalize heading error to -180 to 180 degrees
        if (headingError > 180) headingError -= 360;
        else if (headingError < -180) headingError += 360;
        
        timer.reset();
        
        // Loop until we reach the target position or timeout
        while (opModeIsActive() && 
              (timer.seconds() < timeoutS) && 
              (distanceError > POSITION_THRESHOLD || Math.abs(headingError) > HEADING_THRESHOLD)) {
            
            // Get current position
            currentPosition = odometrySensor.getPosition();
            
            // Calculate distance and heading errors
            distanceError = Math.hypot(targetX - currentPosition.x, targetY - currentPosition.y);
            headingError = targetHeading - currentPosition.h;
            
            // Normalize heading error to -180 to 180 degrees
            if (headingError > 180) headingError -= 360;
            else if (headingError < -180) headingError += 360;
            
            // Calculate the angle to the target point
            double angleToTarget = Math.toDegrees(Math.atan2(targetY - currentPosition.y, targetX - currentPosition.x));
            
            // Normalize the angle to the same range as the heading
            if (angleToTarget < 0) angleToTarget += 360;
            
            // Calculate the robot relative angle (diff between current heading and angle to target)
            double relativeAngle = angleToTarget - currentPosition.h;
            
            // Normalize to -180 to 180
            if (relativeAngle > 180) relativeAngle -= 360;
            else if (relativeAngle < -180) relativeAngle += 360;
            
            // Calculate drive components based on distance and angle
            double driveMagnitude = Math.min(MAX_DRIVE_POWER, P_DRIVE_COEFF * distanceError);
            double driveX = driveMagnitude * Math.cos(Math.toRadians(relativeAngle));
            double driveY = driveMagnitude * Math.sin(Math.toRadians(relativeAngle));
            
            // Calculate rotational power for heading correction
            double turn = (headingError * P_DRIVE_COEFF);
            turn = Math.max(-0.3, Math.min(0.3, turn));  // Limit maximum turn power
            
            // Set motors using the mecanum drive kinematics
            setAllMotorPowers(driveX, driveY, turn);
            
            // Display position data
            telemetry.addData("Target", "X: %.2f, Y: %.2f, H: %.2f", targetX, targetY, targetHeading);
            telemetry.addData("Current", "X: %.2f, Y: %.2f, H: %.2f", 
                             currentPosition.x, currentPosition.y, currentPosition.h);
            telemetry.addData("Error", "Dist: %.2f, Heading: %.2f", distanceError, headingError);
            telemetry.addData("Powers", "X: %.2f, Y: %.2f, Turn: %.2f", driveX, driveY, turn);
            telemetry.update();
        }
        
        // Stop all motors
        setAllMotorPowers(0, 0, 0);
    }
    
    /**
     * Set all motors using mecanum drive kinematics
     */
    private void setAllMotorPowers(double driveX, double driveY, double turn) {
        // Calculate wheel powers using mecanum kinematics
        double frontLeftPower = driveY + driveX + turn;
        double frontRightPower = driveY - driveX - turn;
        double backLeftPower = driveY - driveX + turn;
        double backRightPower = driveY + driveX - turn;
        
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
        
        // Set motor powers
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }
}