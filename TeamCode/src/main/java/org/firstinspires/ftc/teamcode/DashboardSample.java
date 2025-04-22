package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Sample OpMode that demonstrates FTC Dashboard integration
 */
@TeleOp(name = "Dashboard Sample", group = "Samples")
public class DashboardSample extends LinearOpMode {
    
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    
    @Override
    public void runOpMode() {
        // Create dashboard telemetry
        Telemetry dashboardTelemetry = DashboardInitialization.createTelemetry(telemetry);
        
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        
        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Wait for the game to start (driver presses PLAY)
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
        waitForStart();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left stick to go forward, backward, and turn
            double drive = -gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            
            // Calculate wheel powers
            double leftFrontPower = drive + turn + strafe;
            double rightFrontPower = drive - turn - strafe;
            double leftBackPower = drive + turn - strafe;
            double rightBackPower = drive - turn + strafe;
            
            // Normalize wheel powers
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            
            // Set motor powers
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            
            // Show motor powers in telemetry (both on driver station and dashboard)
            dashboardTelemetry.addData("Status", "Running");
            dashboardTelemetry.addData("Left Front Power", "%.2f", leftFrontPower);
            dashboardTelemetry.addData("Right Front Power", "%.2f", rightFrontPower);
            dashboardTelemetry.addData("Left Back Power", "%.2f", leftBackPower);
            dashboardTelemetry.addData("Right Back Power", "%.2f", rightBackPower);
            dashboardTelemetry.update();
        }
    }
}