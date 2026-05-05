package org.firstinspires.ftc.teamcode.common.shooter;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;

import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;
import java.util.List;
import org.firstinspires.ftc.teamcode.configurables.ShooterConfigurables;

public class TeleOpShooter {
    private final BjornHardware hardware;
    private AprilTagCamera camera;
    private int goalTagId = -1;

    private boolean active = false;
    private boolean idle = false;
    private boolean useCv = false;
    // Continuous Ranging State
    private double smoothedVoltage = 0.0;
    private boolean firstReading = true;

    private int targetRpm = 0;
    private double currentDistanceInches = 0.0;
    private double filteredRpm = 0.0;
    private static final double TICKS_PER_REV = 28.0;

    public TeleOpShooter(BjornHardware hardware) {
        this.hardware = hardware;
    }

    public void setCamera(AprilTagCamera camera, int goalTagId) {
        this.camera = camera;
        this.goalTagId = goalTagId;
    }



    public void toggle() {
        if (active) {
            // Turning OFF
            active = false;
        } else {
            // Turning ON
            active = true;
            idle = false;
            firstReading = true; // Reset smoother on activation
        }
    }

    public void toggleIdle() {
        idle = !idle;
        // If we toggle idle ON while active, does nothing (active overrides).
        // If we toggle idle OFF while active, does nothing.
    }

    public void setIdle(boolean isIdle) {
        this.idle = isIdle;
    }

    public void toggleCv() {
        useCv = !useCv;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isIdle() {
        return idle;
    }
    
    public boolean isScanning() {
        // If active, we are scanning/shooting
        return active;
    }
    
    public void startScan() {
        // Enable Active Mode (Scan & Shoot)
        if (!active) {
            active = true;
            idle = false;
            firstReading = true;
        }
    }

    public boolean isCvEnabled() {
        return useCv;
    }

    public int getTargetRpm() {
        return targetRpm;
    }

    public double getMeasuredRpm() {
        return filteredRpm;
    }

    public double getDistanceInches() {
        return currentDistanceInches;
    }

    public boolean isReady() {
        return active && targetRpm > 0 && Math.abs(filteredRpm - targetRpm) < ShooterConfigurables.readyTolRpm;
    }

    public void update() {
        // 1. Update Measured RPM
        double measured = readRpm();
        filteredRpm = (ShooterConfigurables.rpmEmaAlpha * measured) + ((1.0 - ShooterConfigurables.rpmEmaAlpha) * filteredRpm);

        if (active) {
            // Continuous Ranging & RPM Update
            if (hardware.swyftRanger != null) {
                double rawVolts = hardware.swyftRanger.getVoltage();
                
                // Init smoother if first run
                if (firstReading) {
                    smoothedVoltage = rawVolts;
                    firstReading = false;
                } else {
                    smoothedVoltage = (ShooterConfigurables.voltageAlpha * rawVolts) + ((1.0 - ShooterConfigurables.voltageAlpha) * smoothedVoltage);
                }

                // Read Battery for Compensation
                double battV = 13.0; // Default nominal
                if (hardware.batterySensor != null) {
                    try { battV = hardware.batterySensor.getVoltage(); } catch (Exception ignored) {}
                }

                // Calculate Distance
                double distIn = org.firstinspires.ftc.teamcode.common.SwyftRangerConstants.voltageToInches(smoothedVoltage, battV);
                currentDistanceInches = distIn;

                // Calculate RPM (Range -> RPM)
                double rangeFt = distIn / 12.0;
                double slope = ShooterConfigurables.rpmSlopeRanger;
                double offset = ShooterConfigurables.rpmOffsetRanger;
                double calculatedRpm = (slope * rangeFt) + offset;

                // Set Target
                targetRpm = (int) Math.max(ShooterConfigurables.minRpm,
                        Math.min(calculatedRpm, ShooterConfigurables.maxRpm));
            } else {
                 // Fallback if sensor missing but active
                 targetRpm = ShooterConfigurables.idleRpm; 
            }

            setFlywheelRpm(targetRpm);
        } else if (idle) {
            targetRpm = ShooterConfigurables.idleRpm;
            setFlywheelRpm(targetRpm);
        } else {
            targetRpm = 0;
            setFlywheelRpm(0);
        }

        // 5. Update LEDs
        updateLeds();
    }

    public void setTargetRpm(int rpm) {
        this.targetRpm = rpm;
        setFlywheelRpm(rpm);
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

    private void updateLeds() {
        boolean ready = isReady();
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
            // If IDLE, maybe Yellow or Just Red? Usually Red implies "Don't Shoot".
            // Since Idle is just keeping it warm, Red is appropriate.
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
