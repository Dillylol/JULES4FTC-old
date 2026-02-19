package org.firstinspires.ftc.teamcode.common.turret;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.configurables.TurretConfigurables;

/**
 * Calculates turret angles to aim at a fixed goal position.
 * Uses robot's current pose and known goal coordinates to compute aim angle.
 */
public class GoalCalculator {
    private final Pose goalPosition;
    
    /**
     * Creates a goal calculator.
     * @param goalX Goal X coordinate in field reference frame
     * @param goalY Goal Y coordinate in field reference frame
     */
    public GoalCalculator(double goalX, double goalY) {
        this.goalPosition = new Pose(goalX, goalY, 0);
    }
    
    /**
     * Calculates the turret angle needed to aim at the goal from the current robot pose.
     * @param currentPose Current robot pose (x, y, heading)
     * @return Turret angle in degrees (robot-relative, 0-360 range clipped to limits)
     */
    public double getTurretAngleToGoal(Pose currentPose) {
        // Vector from robot to goal
        double dx = goalPosition.getX() - currentPose.getX();
        double dy = goalPosition.getY() - currentPose.getY();
        
        // Angle to goal in field frame (radians)
        double angleToGoalRad = Math.atan2(dy, dx);
        double angleToGoalDeg = Math.toDegrees(angleToGoalRad);
        
        // Convert to robot-relative angle
        // Robot heading is in radians, convert to degrees
        double robotHeadingDeg = Math.toDegrees(currentPose.getHeading());
        double turretAngle = angleToGoalDeg - robotHeadingDeg;
        
        // Normalize to 0-360 range
        while (turretAngle < 0) turretAngle += 360;
        while (turretAngle >= 360) turretAngle -= 360;
        
        // Clip to turret limits
        return Range.clip(turretAngle, 
            TurretConfigurables.limitMin, 
            TurretConfigurables.limitMax);
    }
    
    /**
     * Gets the distance to the goal from the current position.
     * @param currentPose Current robot pose
     * @return Distance to goal in inches
     */
    public double getDistanceToGoal(Pose currentPose) {
        double dx = goalPosition.getX() - currentPose.getX();
        double dy = goalPosition.getY() - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}
