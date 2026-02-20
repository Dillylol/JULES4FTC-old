package org.firstinspires.ftc.teamcode.common.turret;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.configurables.TurretConfigurables;

/**
 * Calculates turret angles to aim at a fixed goal position.
 * 
 * Uses 3-point kinematics: accounts for the turret's physical offset from the
 * robot's center of rotation, then maps the result into the hardware coordinate
 * frame (0° = Left hardstop, 90° = Center, 180° = Right hardstop).
 * 
 * Math Frame: 0° = Right, 90° = Forward, 180° = Left (standard unit circle)
 * Hardware: 0° = Left, 90° = Center, 180° = Right (physical servo range)
 * Mapping: θ_physical = 90° - θ_relative
 */
public class GoalCalculator {
    private final Pose goalPosition;

    // Turret physical offset from robot center (cached from constants)
    private final double turretOffsetX;
    private final double turretOffsetY;

    /**
     * Creates a goal calculator.
     * 
     * @param goalX Goal X coordinate in field reference frame
     * @param goalY Goal Y coordinate in field reference frame
     */
    public GoalCalculator(double goalX, double goalY) {
        this.goalPosition = new Pose(goalX, goalY, 0);
        this.turretOffsetX = BjornConstants.Motors.TURRET_OFFSET_X;
        this.turretOffsetY = BjornConstants.Motors.TURRET_OFFSET_Y;
    }

    /**
     * Calculates the turret angle needed to aim at the goal from the current robot
     * pose.
     * 
     * Uses 3-point kinematics to account for turret physical offset, then maps
     * from standard math angles into the hardware's physical servo frame.
     * 
     * @param currentPose Current robot pose (x, y, heading in radians)
     * @return Turret angle in degrees (hardware frame, clipped to limits)
     */
    public double getTurretAngleToGoal(Pose currentPose) {
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double robotHeadingRad = currentPose.getHeading();

        // === Phase 3: 3-Point Kinematic Transform ===
        // Transform turret position from robot-local to field-global coordinates
        double turretX = robotX
                + turretOffsetX * Math.cos(robotHeadingRad)
                - turretOffsetY * Math.sin(robotHeadingRad);
        double turretY = robotY
                + turretOffsetX * Math.sin(robotHeadingRad)
                + turretOffsetY * Math.cos(robotHeadingRad);

        // Vector from TURRET (not robot center) to goal
        double dx = goalPosition.getX() - turretX;
        double dy = goalPosition.getY() - turretY;

        // Field-frame angle to goal (standard math: 0° = Right, CCW positive)
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // === Phase 4: Coordinate Frame Mapping ===
        // Step 1: Convert to robot-relative angle
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);
        double relativeAngle = fieldAngleDeg - robotHeadingDeg;

        // Normalize to shortest path (-180 to 180)
        while (relativeAngle > 180)
            relativeAngle -= 360;
        while (relativeAngle <= -180)
            relativeAngle += 360;

        // Step 2: Map from math frame to hardware frame
        // Math: 0° = Right, 90° = Forward, 180° = Left (CCW positive)
        // Hardware: 0° = Left, 90° = Center, 180° = Right (CW positive)
        // Mapping: θ_physical = 90° - θ_relative
        double physicalAngle = 90.0 - relativeAngle;

        // Clip to turret hardware limits
        return Range.clip(physicalAngle,
                TurretConfigurables.limitMin,
                TurretConfigurables.limitMax);
    }

    /**
     * Gets the distance to the goal from the current position.
     * Uses turret position (with offset) for accuracy.
     * 
     * @param currentPose Current robot pose
     * @return Distance to goal in inches
     */
    public double getDistanceToGoal(Pose currentPose) {
        double robotHeadingRad = currentPose.getHeading();

        double turretX = currentPose.getX()
                + turretOffsetX * Math.cos(robotHeadingRad)
                - turretOffsetY * Math.sin(robotHeadingRad);
        double turretY = currentPose.getY()
                + turretOffsetX * Math.sin(robotHeadingRad)
                + turretOffsetY * Math.cos(robotHeadingRad);

        double dx = goalPosition.getX() - turretX;
        double dy = goalPosition.getY() - turretY;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
