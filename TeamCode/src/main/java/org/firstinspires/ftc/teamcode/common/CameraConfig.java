package org.firstinspires.ftc.teamcode.common;

/**
 * Centralized camera + AprilTag configuration for the 2024-2025 DECODE field.
 * Tunable values can be edited and will persist during the app session.
 */
public final class CameraConfig {

    // Physical camera reference
    public static final String WEBCAM_NAME = "Webcam 1";

    // Tag IDs for the season
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int OBELISK_TAG_ID_21 = 21;
    public static final int OBELISK_TAG_ID_22 = 22;
    public static final int OBELISK_TAG_ID_23 = 23;
    public static final int RED_GOAL_TAG_ID = 24;

    // Camera intrinsics for Logitech C920 at 1280x720
    // Camera intrinsics for Logitech C920 at 1280x720
    // Tunable via CameraConfigurables/Dashboard
    public static double FX = 930.0;
    public static double FY = 930.0;
    public static double CX = 640.0;
    public static double CY = 360.0;

    // --- Tunable Camera Settings (Logitech C920 @ 640x480) ---
    // Strategy: Minimum exposure to freeze motion, gain compensates brightness.
    // Tune with ConceptAprilTagOptimizeExposure if lighting changes.
    public static int TUNED_EXPOSURE = 6; // ms — lowest that still detects tags at range
    public static int TUNED_GAIN = 200; // Lower than 250 to reduce noise; increase if tags lost in dim light
    public static int TUNED_DECIMATION = 2; // 2 = better detection with motion blur (was 3)

    // FTC tag size (6.5 inches)
    public static final double TAG_SIZE_METERS = 0.165;

    public static final String CLASS_BLUE_GOAL = "blue_goal";
    public static final String CLASS_RED_GOAL = "red_goal";
    public static final String CLASS_OBELISK = "obelisk";
    public static final String CLASS_UNKNOWN = "unknown";

    // --- Control Mapping ---
    // 1.0 = Add yaw to current. -1.0 = Subtract yaw from current.
    // Use this to flip direction if turret moves AWAY from tag.
    public static final double CAMERA_TO_TURRET_SCALAR = 1.0;
    
    // Lateral offset of camera relative to robot/turret center.
    // Positive = Camera is to the LEFT of center relative to the shooter.
    public static final double CAMERA_LATERAL_OFFSET_INCHES = 2.0;

    private CameraConfig() {
    }

    public static String classify(int id) {
        if (id == BLUE_GOAL_TAG_ID) {
            return CLASS_BLUE_GOAL;
        }
        if (id == RED_GOAL_TAG_ID) {
            return CLASS_RED_GOAL;
        }
        if (id == OBELISK_TAG_ID_21 || id == OBELISK_TAG_ID_22 || id == OBELISK_TAG_ID_23) {
            return CLASS_OBELISK;
        }
        return CLASS_UNKNOWN;
    }

    public static boolean isGoalTag(int id) {
        return id == BLUE_GOAL_TAG_ID || id == RED_GOAL_TAG_ID;
    }
}
