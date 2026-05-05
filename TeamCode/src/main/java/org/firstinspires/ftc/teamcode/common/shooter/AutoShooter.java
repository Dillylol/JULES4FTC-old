package org.firstinspires.ftc.teamcode.common.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.configurables.ShooterConfigurables;

public class AutoShooter {
    private final BjornHardware hardware;
    private int targetRpm = 0;
    private double filteredRpm = 0.0;
    private boolean isShooting = false;
    private static final double TICKS_PER_REV = 28.0;

    public AutoShooter(BjornHardware hardware) {
        this.hardware = hardware;
    }

    public void setTargetRpm(int rpm) {
        // Clamp to safe range if non-zero
        if (rpm > 0) {
            this.targetRpm = Math.max((int) ShooterConfigurables.minRpm,
                    Math.min(rpm, (int) ShooterConfigurables.maxRpm));
        } else {
            this.targetRpm = 0;
        }
    }

    public void setTargetDistance(double distanceInches) {
        double rangeFt = distanceInches / 12.0;
        double slope = ShooterConfigurables.rpmSlopeRanger;
        double offset = ShooterConfigurables.rpmOffsetRanger;
        double calculatedRpm = (slope * rangeFt) + offset;
        setTargetRpm((int) calculatedRpm);
    }

    public void setIntakePower(boolean on) {
        if (hardware.intake != null) {
            hardware.intake.setPower(on ? 1.0 : 0.0);
        }
    }

    public void setShooting(boolean shooting) {
        this.isShooting = shooting;
    }

    public boolean isReady() {
        if (targetRpm == 0)
            return false;
        return Math.abs(filteredRpm - targetRpm) < ShooterConfigurables.readyTolRpm;
    }

    public void update() {
        // 1. Calculate RPM
        double measured = readRpm();
        filteredRpm = (ShooterConfigurables.rpmEmaAlpha * measured) + ((1.0 - ShooterConfigurables.rpmEmaAlpha) * filteredRpm);

        // 2. Battery Compensation
        int commandRpm = targetRpm;
        if (targetRpm > 0 && hardware.batterySensor != null) {
            double voltage = hardware.getBatteryVoltage();
            double nominal = ShooterConfigurables.nominalBattV;
            // Simple Feedforward compensation: if voltage drops, increase RPM target
            // slightly to maintain speed
            // This is a basic P-like adjustment based on voltage sag
            double sag = Math.max(0, nominal - voltage);
            // K_V is defined in configurables, usually 0 but allows tuning
            commandRpm += (int) (sag * ShooterConfigurables.shooterKVRpm);
        }

        // 3. Set Motor Powers/Velocity
        setFlywheelRpm(commandRpm);

        // 4. Handle Ready State (LEDs + Auto Grip)
        boolean ready = isReady();
        updateLeds(ready);

        if (isShooting) {
            // Auto Grip Feed
            if (hardware.grip1 != null)
                hardware.grip1.setPower(1.0);
            if (hardware.grip2 != null)
                hardware.grip2.setPower(1.0);
        } else {
            // Stop grips if not ready (unless intake overrides, but AutoShooter owns this
            // now for Auto)
            // In Auto, usually we want to stop feeding if RPM drops.
            if (hardware.grip1 != null)
                hardware.grip1.setPower(0.0);
            if (hardware.grip2 != null)
                hardware.grip2.setPower(0.0);
        }
    }

    private void setFlywheelRpm(int rpm) {
        double ticksPerSec = (rpm * TICKS_PER_REV) / 60.0;
        if (hardware.wheel != null)
            hardware.wheel.setVelocity(ticksPerSec);
        if (hardware.wheel2 != null)
            hardware.wheel2.setVelocity(ticksPerSec);
    }

    private double readRpm() {
        if (hardware.wheel == null)
            return 0.0;
        double v1 = hardware.wheel.getVelocity();
        double v2 = (hardware.wheel2 != null) ? hardware.wheel2.getVelocity() : v1;
        double avgV = (v1 + v2) / 2.0;
        return (avgV / TICKS_PER_REV) * 60.0;
    }

    private void updateLeds(boolean ready) {
        if (ready) {
            if (hardware.led1Green != null)
                hardware.led1Green.setState(true);
            if (hardware.led1Red != null)
                hardware.led1Red.setState(false);
            if (hardware.led2Green != null)
                hardware.led2Green.setState(true);
            if (hardware.led2Red != null)
                hardware.led2Red.setState(false);
        } else {
            // Not ready (Red)
            if (hardware.led1Green != null)
                hardware.led1Green.setState(false);
            if (hardware.led1Red != null)
                hardware.led1Red.setState(true);
            if (hardware.led2Green != null)
                hardware.led2Green.setState(false);
            if (hardware.led2Red != null)
                hardware.led2Red.setState(true);
        }
    }
}
