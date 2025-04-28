package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

/**
 * Utility functions for interacting with Lynx Modules (Control/Expansion Hubs).
 */
public class LynxModuleUtil {
    // Map of module serial numbers to firmware versions
    private static final Map<Integer, String> FIRMWARE_VERSION_CACHE = new HashMap<>();

    /**
     * Minimum firmware version required for optimal operation
     */
    public static final String MIN_VERSION = "1.8.2";

    /**
     * Ensures all the REV hubs have at least the minimum firmware version.
     * Shows a warning message if any hub's firmware is outdated.
     *
     * @param hardwareMap reference to the hardware map
     */
    public static void ensureMinimumFirmwareVersion(HardwareMap hardwareMap) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            int moduleNumber = module.getModuleAddress();
            
            // Check if we've already seen this module
            if (!FIRMWARE_VERSION_CACHE.containsKey(moduleNumber)) {
                // Get the firmware version of this module
                String firmware = module.getFirmwareVersionString();
                FIRMWARE_VERSION_CACHE.put(moduleNumber, firmware);
                
                // Display a warning if firmware is outdated
                if (compareVersions(firmware, MIN_VERSION) < 0) {
                    System.out.println("⚠️ Outdated Expansion Hub firmware detected on module " + moduleNumber);
                    System.out.println("Current version: " + firmware);
                    System.out.println("Minimum version: " + MIN_VERSION);
                    System.out.println("Please update using the REV Hub Firmware Update tool");
                }
            }
        }
    }
    
    /**
     * Compares two version strings.
     * 
     * @param version1 first version string
     * @param version2 second version string
     * @return a negative integer if version1 < version2, zero if version1 == version2, 
     *         or a positive integer if version1 > version2
     */
    private static int compareVersions(String version1, String version2) {
        if (version1 == null || version2 == null) {
            return 0;
        }
        
        String[] components1 = version1.split("\\.");
        String[] components2 = version2.split("\\.");
        int length = Math.min(components1.length, components2.length);
        
        for (int i = 0; i < length; i++) {
            int c1 = Integer.parseInt(components1[i]);
            int c2 = Integer.parseInt(components2[i]);
            
            if (c1 != c2) {
                return c1 - c2;
            }
        }
        
        return components1.length - components2.length;
    }
}
