package org.firstinspires.ftc.teamcode.jules.shot;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.BjornHardware;

/**
 * Compatibility wrapper that delegates to common shooter implementation.
 */
public final class ShooterController {
    private final org.firstinspires.ftc.teamcode.common.shooter.ShooterController delegate;

    public static final class ShotMetrics {
        public final double rpmAtFire;
        public final long timeToReadyMs;
        public final long fireTimestampMs;

        ShotMetrics(double rpmAtFire, long timeToReadyMs, long fireTimestampMs) {
            this.rpmAtFire = rpmAtFire;
            this.timeToReadyMs = timeToReadyMs;
            this.fireTimestampMs = fireTimestampMs;
        }
    }

    public ShooterController(@Nullable DcMotorEx flywheel,
            @Nullable DcMotorEx flywheelSecondary,
            @Nullable DcMotorEx intake,
            @Nullable BjornHardware hardware) {
        this.delegate = new org.firstinspires.ftc.teamcode.common.shooter.ShooterController(
                flywheel, flywheelSecondary, intake, hardware);
    }

    public ShooterController(@Nullable DcMotorEx flywheel,
            @Nullable DcMotorEx flywheelSecondary,
            @Nullable DcMotorEx intake,
            @Nullable VoltageSensor vSensor) {
        this.delegate = new org.firstinspires.ftc.teamcode.common.shooter.ShooterController(
                flywheel, flywheelSecondary, intake, null, vSensor);
    }

    public void setTargetRpm(double rpm, long nowMs) {
        delegate.setTargetRpm(rpm, nowMs);
    }

    public int getTargetRpm() {
        return delegate.getTargetRpm();
    }

    public void update(long nowMs) {
        delegate.update(nowMs);
    }

    public boolean isReady(long nowMs) {
        return delegate.isReady(nowMs);
    }

    public boolean fire(long nowMs) {
        return delegate.fire(nowMs);
    }

    public void stop(long nowMs) {
        delegate.stop(nowMs);
    }

    public double getRpmEstimate() {
        return delegate.getRpmEstimate();
    }

    public ShotMetrics pollShotMetrics() {
        org.firstinspires.ftc.teamcode.common.shooter.ShooterController.ShotMetrics m = delegate.pollShotMetrics();
        if (m == null) {
            return null;
        }
        return new ShotMetrics(m.rpmAtFire, m.timeToReadyMs, m.fireTimestampMs);
    }
}
