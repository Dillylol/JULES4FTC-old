
package org.firstinspires.ftc.teamcode.configurables;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/**
 * Turret Configuration - Tunable via Panels UI
 * 
 * Based on proven stable values from OldPrograms (75.52:1 ratio, PD control)
 * These settings can be adjusted in real-time through the Panels dashboard.
 */
@Configurable
public class TurretConfigurables {

    // === Hardware Configuration ===
    
    @Sorter(sort = 0)
    public static double motorTicksPerRev = 28.0;
    
    @Sorter(sort = 1)
    public static double gearReduction = 75.52; // Old stable ratio (5.23 × 3.61 × 4)
    
    // === PD Control (No Integral - proven stable) ===
    
    @Sorter(sort = 10)
    public static double kP = 0.015;

    @Sorter(sort = 11)
    public static double kD = 0.004;
    
    @Sorter(sort = 12)
    public static double robotRotationFFGain = 0.0055;

    @Sorter(sort = 13)
    public static double trackerFFGain = 0.006; // Specific sensitivity for Pedro tracking
    
    // === Behavior ===
    
    @Sorter(sort = 20)
    public static double deadband = 2.0; // Degrees - prevents oscillation
    
    @Sorter(sort = 21)
    public static double maxPowerDelta = 0.15; // Slew rate limit
    
    @Sorter(sort = 22)
    public static double forcefieldPower = 0.6; // Immediate hard push at limits
    
    // === Limits ===
    
    @Sorter(sort = 30)
    public static double limitMin = -10.0;
    
    @Sorter(sort = 31)
    public static double limitMax = 155.0;
    
    // === Manual Control ===
    
    @Sorter(sort = 40)
    public static double manualRateDegPerSec = 90.0;
    
    @Sorter(sort = 41)
    public static double g2TurretPower = 0.75; // Gamepad 2 authority
    
    @Sorter(sort = 42)
    public static double g1TurretPower = 0.5; // Gamepad 1 authority
    
    // === AprilTag Tracking ===
    
    @Sorter(sort = 50)
    public static double aprilTagCorrectionGain = 0.5;

    // === Camera Tracking PID (separate from base turret PD) ===
    
    @Sorter(sort = 60)
    public static double cameraKp = 0.012;

    @Sorter(sort = 61)
    public static double cameraKd = 0.003;

    @Sorter(sort = 62)
    public static double cameraCorrectionGain = 1.0; // Scale factor for camera bearing error
    
    // Calculated property (DO NOT TUNE DIRECTLY)
    public static double getTicksPerDegree() {
        return (motorTicksPerRev * gearReduction) / 360.0;
    }
}
