package org.firstinspires.ftc.teamcode.common.turret;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

/**
 * Logic-only class to calculate the desired turret angle (0-180)
 * based on Robot Pose and Target Location.
 * Implements "Hemispherical" constraints with memory.
 */
public class HemisphericalTurret {

    private double lastValidEdge = 90.0; // Default center
    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 180.0;

    /**
     * Calculates the `targetFieldHeading` compatible with BjornTeleBlue's control loop.
     * 
     * In BjornTeleBlue logic:
     * desiredPhysicalAngle = targetFieldHeading + robotYaw
     * Therefore:
     * targetFieldHeading = desiredPhysicalAngle - robotYaw
     * 
     * @param robotPose Current Pedro Robot Pose (x, y, heading in RADIANS)
     * @param target Field Point to look at (As Pose, heading ignored)
     * @param currentTurretAngle Current physical turret angle (0-180)
     * @return The calculated `targetFieldHeading` to feed into updateTurret()
     */
    public double calculate(Pose robotPose, Pose target, double currentTurretAngle) {
        
        // 1. Calculate Absolute Bearing to Target from Robot
        double dy = target.getY() - robotPose.getY();
        double dx = target.getX() - robotPose.getX();
        double targetBearingRad = Math.atan2(dy, dx); // Field absolute angle
        
        // 2. Calculate Desired Relative Angle (The angle the turret SHOULD physically be at)
        // Robot Heading (Pedro uses standard unit circle: 0 = East, 90 = North)
        // Relative = Target - RobotHeading
        double relativeAngleRad = targetBearingRad - robotPose.getHeading();
        
        // Normalize to -PI to PI
        while (relativeAngleRad > Math.PI) relativeAngleRad -= 2 * Math.PI;
        while (relativeAngleRad <= -Math.PI) relativeAngleRad += 2 * Math.PI;
        
        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);
        
        // 3. Coordinate System Correction
        // We need to know where 0 and 180 are on the robot.
        // Assuming 0 = Right, 180 = Left, 90 = Front.
        // Pedro Heading is normally "Front"? 
        // If Pedro Heading 0 is East, and Robot is facing East:
        // Target East (0) -> Abs 0 -> Rel 0.
        // If Turret 0 is "Right" (-90 relative to front), we need offset.
        // Let's assume standard: 0 is Right Side of robot. 90 is Front.
        // If robot faces target: relativeAngle is 0 (Front).
        // Turret should be at 90.
        // So `desiredPhysical` = `relativeAngleDeg` + 90.
        // Test: Target is Right (Relative -90). Desired = -90 + 90 = 0. Correct.
        // Test: Target is Left (Relative +90). Desired = 90 + 90 = 180. Correct.
        
        double desiredPhysical = relativeAngleDeg + 90.0;
        
        // 4. Hemispherical Logic (The "Memory" Part)
        // If desiredPhysical is within 0-180, we go there.
        // If it is OUTSIDE (e.g. -10 or 190), we clip to the nearest edge (0 or 180)
        // AND we hold that edge until it comes back into view.
        
        if (desiredPhysical >= MIN_ANGLE && desiredPhysical <= MAX_ANGLE) {
            // Valid Zone
            lastValidEdge = desiredPhysical;
        } else {
            // Invalid Zone (Behind robot)
            // Determine which side implies "closest path" or "waiting at the border"
            
            // If we are at -20, we should stay at 0.
            // If we are at 200, we should stay at 180.
            // Simple clamp works for "Stick to Edge", but we need to ensure we don't flip 
            // all the way around if it crosses the back-dead-center.
            
            if (desiredPhysical < MIN_ANGLE) {
                 // It's to the right/back. Clamp 0.
                 lastValidEdge = MIN_ANGLE;
            } else if (desiredPhysical > MAX_ANGLE) {
                 // It's to the left/back. Clamp 180.
                 lastValidEdge = MAX_ANGLE;
            }
        }
        
        double finalPhysicalTarget = lastValidEdge;
        
        // 5. Convert back to `targetFieldHeading` for BjornTeleBlue logic
        // logic: targetFieldHeading = desiredPhysicalAngle - robotYaw(Degrees)
        
        double robotYawDeg = Math.toDegrees(robotPose.getHeading());
        return finalPhysicalTarget - robotYawDeg;
    }
}
