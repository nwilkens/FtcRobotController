package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Utility class for initializing and using FTC Dashboard
 */
public class DashboardInitialization {
    
    private static FtcDashboard dashboard = FtcDashboard.getInstance();
    
    /**
     * Get a telemetry object that sends data to both the driver station and the dashboard
     * @param telemetry The original telemetry from the OpMode
     * @return A MultipleTelemetry instance that sends data to both the driver station and dashboard
     */
    public static Telemetry createTelemetry(Telemetry telemetry) {
        return new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }
    
    /**
     * Get the dashboard instance
     * @return The FTC Dashboard instance
     */
    public static FtcDashboard getDashboard() {
        return dashboard;
    }
}