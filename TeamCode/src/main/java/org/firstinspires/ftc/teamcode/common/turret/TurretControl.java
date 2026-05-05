package org.firstinspires.ftc.teamcode.common.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.configurables.TurretConfigurables;

/**
 * Simplified Turret Control with PD Loop and Forcefield Limits
 * 
 * Based on proven stable implementation from OldPrograms.
 * Features:
 * - Direct encoder tracking (no estimator abstraction)
 * - Simple PD control (no integral term)
 * - Forcefield override at limits (bypasses slew rate for immediate correction)
 * - Position-based tracking for auto targeting
 * - Reusable from TeleOp and Auto programs
 * 
 * Usage:
 * 1. Create instance: new TurretControl(turretMotor, imu)
 * 2. Call update(dt) every loop
 * 3. Set target via setTargetAngle() or enable position tracking via
 * setTargetPosition()
 */
public class TurretControl {

    private final DcMotorEx turretMotor;
    private final IMU imu;

    // === State Variables ===
    private double turretAngleDeg = 0.0; // Current turret angle (local frame)
    private double targetFieldHeading = 90.0; // Target in field/world frame
    private int lastEncoderPos = 0;
    private double lastTurretPower = 0.0;
    private double lastTurretError = 0.0;
    private double lastRobotYaw = 0.0;

    public enum TargetMode {
        LOCAL, // Target is robot-relative
        WORLD // Target is field-relative (compensates for robot yaw)
    }

    private TargetMode targetMode = TargetMode.LOCAL;

    // === Position Tracking State ===
    private Double targetPositionX = null;
    private Double targetPositionY = null;
    private com.pedropathing.geometry.Pose currentRobotPose = null;
    private boolean positionTrackingEnabled = false;

    /**
     * Constructs a new TurretControl instance.
     * 
     * @param turretMotor The turret motor (must have encoder)
     * @param imu         The robot's IMU for heading tracking
     */
    public TurretControl(DcMotorEx turretMotor, IMU imu) {
        this.turretMotor = turretMotor;
        this.imu = imu;
        this.lastEncoderPos = turretMotor.getCurrentPosition();
    }

    /**
     * Updates the turret control loop. Call this every loop iteration.
     * 
     * @param dt Delta time since last update (seconds)
     * @return The calculated motor power (already applied internally)
     */
    public double update(double dt) {
        // === Update Turret Angle from Encoder ===
        int currentPos = turretMotor.getCurrentPosition();
        int deltaTicks = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;
        turretAngleDeg += (deltaTicks / TurretConfigurables.getTicksPerDegree())
                * BjornConstants.Motors.TURRET_ENCODER_DIRECTION;

        // === Get Robot Rotation ===
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double robotRate = (dt > 0) ? (robotYaw - lastRobotYaw) / dt : 0.0;
        lastRobotYaw = robotYaw;

        // Position tracking math is now handled exclusively by GoalCalculator
        // in the TeleOp loop. TurretControl only executes the target it receives.

        // === PD Control (World-frame target → Local-frame control) ===
        // === PD Control (Apply Target Mode) ===
        double desiredTurretAngle;

        if (targetMode == TargetMode.WORLD) {
            // World Frame: Target - RobotYaw (Subtractive Counter-Rotation)
            desiredTurretAngle = targetFieldHeading - robotYaw;
        } else {
            // Local Frame: Direct target
            desiredTurretAngle = targetFieldHeading;
        }

        // ABSOLUTE SAFETY CLAMP: Ensure target never exceeds physical limits
        desiredTurretAngle = Range.clip(desiredTurretAngle,
                TurretConfigurables.limitMin,
                TurretConfigurables.limitMax);

        double error = desiredTurretAngle - turretAngleDeg;

        // Deadband: Prevent oscillation
        if (Math.abs(error) < TurretConfigurables.deadband) {
            error = 0;
        }

        // PD calculation - use camera PID when tracking, base PID otherwise
        double derivative = (dt > 0) ? (error - lastTurretError) / dt : 0.0;
        double activeKp = positionTrackingEnabled ? TurretConfigurables.cameraKp : TurretConfigurables.kP;
        double activeKd = positionTrackingEnabled ? TurretConfigurables.cameraKd : TurretConfigurables.kD;
        double pid = (error * activeKp) + (derivative * activeKd);

        // Use specific FF gain for tracking mode if enabled
        double ffGain = positionTrackingEnabled ? TurretConfigurables.trackerFFGain
                : TurretConfigurables.robotRotationFFGain;
        double ff = robotRate * ffGain;

        double turretPower = pid + ff;
        lastTurretError = error;

        // === Standard Limit Enforcement ===
        if (turretAngleDeg < TurretConfigurables.limitMin && turretPower < 0)
            turretPower = 0;
        if (turretAngleDeg > TurretConfigurables.limitMax && turretPower > 0)
            turretPower = 0;

        // === Clamp and Slew Rate Limit ===
        if (Double.isNaN(turretPower))
            turretPower = 0;
        turretPower = Range.clip(turretPower, -TurretConfigurables.g2TurretPower,
                TurretConfigurables.g2TurretPower);

        double powerDelta = Range.clip(turretPower - lastTurretPower,
                -TurretConfigurables.maxPowerDelta,
                TurretConfigurables.maxPowerDelta);
        double slewPower = lastTurretPower + powerDelta;

        // === FORCEFIELD OVERRIDE (Bypasses Slew Rate) ===
        // If we are out of bounds, apply IMMEDIATE corrective force
        if (turretAngleDeg < TurretConfigurables.limitMin) {
            slewPower = TurretConfigurables.forcefieldPower; // Hard push right
        } else if (turretAngleDeg > TurretConfigurables.limitMax) {
            slewPower = -TurretConfigurables.forcefieldPower; // Hard push left
        }

        lastTurretPower = slewPower;
        turretMotor.setPower(slewPower * BjornConstants.Motors.TURRET_POWER_DIRECTION);

        return slewPower;
    }

    // === Target Setting Methods ===

    /**
     * Sets the target angle. Interpretation depends on TargetMode.
     * 
     * @param angle Target angle in degrees
     */
    public void setTargetAngle(double angle) {
        this.targetFieldHeading = angle;
    }

    /**
     * Sets the target mode (LOCAL or WORLD).
     * 
     * @param mode TargetMode.LOCAL or TargetMode.WORLD
     */
    public void setTargetMode(TargetMode mode) {
        this.targetMode = mode;
    }

    /**
     * Gets the current target angle in field coordinates.
     * 
     * @return Target angle in degrees
     */
    public double getTargetAngle() {
        return targetFieldHeading;
    }

    /**
     * Sets the target position in field coordinates for position-based tracking.
     * Call setPositionTrackingEnabled(true) to activate.
     * 
     * @param x Target X coordinate in inches
     * @param y Target Y coordinate in inches
     */
    public void setTargetPosition(double x, double y) {
        this.targetPositionX = x;
        this.targetPositionY = y;
    }

    /**
     * Updates the current robot pose from Pedro Pathing odometry.
     * This should be called every loop to keep tracking accurate.
     * 
     * @param pose Current robot pose from follower.getPose()
     */
    public void updateRobotPose(com.pedropathing.geometry.Pose pose) {
        this.currentRobotPose = pose;
    }

    /**
     * Enables or disables position-based tracking mode.
     * When enabled, turret automatically aims at the target position.
     * 
     * @param enabled True to enable position tracking, false for manual/angle
     *                control
     */
    public void setPositionTrackingEnabled(boolean enabled) {
        this.positionTrackingEnabled = enabled;
    }

    /**
     * Checks if position tracking is currently enabled.
     * 
     * @return True if position tracking is active
     */
    public boolean isPositionTrackingEnabled() {
        return positionTrackingEnabled;
    }

    // === Status Methods ===

    /**
     * Gets the current turret angle in local/robot frame.
     * 
     * @return Current angle in degrees
     */
    public double getCurrentAngle() {
        return turretAngleDeg;
    }

    /**
     * Gets the current error (target - actual).
     * 
     * @return Error in degrees
     */
    public double getError() {
        return lastTurretError;
    }

    /**
     * Checks if the turret is locked on target.
     * 
     * @return True if error is within deadband tolerance
     */
    public boolean isLocked() {
        return Math.abs(lastTurretError) <= TurretConfigurables.deadband;
    }

    /**
     * Calculates the distance to the target position.
     * 
     * @return Distance in inches, or -1 if no target/pose is set
     */
    public double getDistanceToTarget() {
        if (currentRobotPose == null || targetPositionX == null || targetPositionY == null) {
            return -1.0;
        }
        double dx = targetPositionX - currentRobotPose.getX();
        double dy = targetPositionY - currentRobotPose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Resets the turret angle to a known position.
     * Useful after encoder reset or homing.
     * 
     * @param angle The known angle in degrees
     */
    public void resetAngle(double angle) {
        this.turretAngleDeg = angle;
        this.lastTurretError = 0.0;
        this.lastTurretPower = 0.0;
    }
}
