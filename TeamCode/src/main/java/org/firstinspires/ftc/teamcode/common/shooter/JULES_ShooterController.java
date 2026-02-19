package org.firstinspires.ftc.teamcode.common.shooter;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;

/**
 * JULES_ShooterController
 * A comprehensive Shooter Controller combining TeleOp manual control and Auto/RL tuning capabilities.
 * Designed to be used with the RpmRecoveryRL JULES program.
 */
public class JULES_ShooterController {

    public static final class ShotMetrics {
        public final double rpmAtFire;
        public final long timeToReadyMs;
        public final long fireTimestampMs;
        public final double voltageDrop;

        ShotMetrics(double rpmAtFire, long timeToReadyMs, long fireTimestampMs, double voltageDrop) {
            this.rpmAtFire = rpmAtFire;
            this.timeToReadyMs = timeToReadyMs;
            this.fireTimestampMs = fireTimestampMs;
            this.voltageDrop = voltageDrop;
        }
    }

    private final BjornHardware hardware;
    private final DcMotorEx flywheel, flywheelSecondary;
    private final VoltageSensor vSensor;

    // Configuration
    private static final double TICKS_PER_REV = 28.0;
    private static final double READY_TOL_RPM = 100.0;
    private static final long READY_SETTLE_MS = 200L;
    private static final double EMA_ALPHA = 0.2;

    // State
    private int targetRpm = 0;
    private double filteredRpm = 0.0;
    private int commandedRpm = 0;
    private long customRampDurationMs = -1; // -1 means use constant
    
    // Ramp State
    private boolean rampActive = false;
    private int rampStartRpm = 0;
    private long rampStartTimeMs = 0;
    private long targetSetTimeMs = 0;

    // Ready State
    private boolean readyLatched = false;
    private long readyStartMs = 0;
    private long readyAtMs = 0;
    
    // Battery Comp
    private boolean batteryCompEnabled = true;

    // Metrics
    private ShotMetrics lastShotMetrics;

    public JULES_ShooterController(BjornHardware hardware) {
        this.hardware = hardware;
        this.flywheel = hardware.wheel;
        this.flywheelSecondary = hardware.wheel2;
        this.vSensor = hardware.batterySensor;
    }

    public void setTargetRpm(int rpm) {
        int safeRpm = (rpm <= 0) ? 0 : Math.max((int)BjornConstants.Power.SHOOTER_MIN_RPM, Math.min(rpm, (int)BjornConstants.Power.SHOOTER_MAX_RPM));
        
        if (this.targetRpm != safeRpm) {
            this.targetRpm = safeRpm;
            this.targetSetTimeMs = System.currentTimeMillis();
            
            // Reset Ramp
            this.rampActive = (safeRpm > 0);
            this.rampStartRpm = (int) filteredRpm;
            this.rampStartTimeMs = System.currentTimeMillis();
            
            // Reset Ready
            this.readyLatched = false;
            this.readyStartMs = 0;
            this.readyAtMs = 0;
        }
    }

    public void setRampDuration(long ms) {
        this.customRampDurationMs = ms;
    }
    
    public void enableBatteryCompensation(boolean enable) {
        this.batteryCompEnabled = enable;
    }

    public int getTargetRpm() {
        return targetRpm;
    }
    
    public double getMeasuredRpm() {
        return filteredRpm;
    }
    
    public boolean isReady() {
         return targetRpm > 0 && readyLatched;
    }
    
    public ShotMetrics getLastShotMetrics() {
        return lastShotMetrics;
    }
    
    public void setFeedforward(double f) {
        if (flywheel == null) return;
        
        // Update both motors if applicable
        // We read the current coefficients first to preserve P, I, D logic if any
        com.qualcomm.robotcore.hardware.PIDFCoefficients pidf = flywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (pidf != null) {
            com.qualcomm.robotcore.hardware.PIDFCoefficients newPidf = 
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(pidf.p, pidf.i, pidf.d, f);
            
            flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, newPidf);
            if (flywheelSecondary != null) {
                flywheelSecondary.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, newPidf);
            }
        }
    }

    public void update() {
        long nowMs = System.currentTimeMillis();
        
        // 1. Read & Filter RPM
        double measured = readRpm();
        filteredRpm = (EMA_ALPHA * measured) + ((1.0 - EMA_ALPHA) * filteredRpm);
        
        // 2. Determine Desired RPM (Target + Battery Comp)
        int desiredCompensated = targetRpm;
        if (targetRpm > 0 && batteryCompEnabled && vSensor != null) {
            double v = vSensor.getVoltage();
            double sag = Math.max(0, BjornConstants.Power.NOMINAL_BATT_V - v);
            desiredCompensated += (int)(sag * BjornConstants.Power.SHOOTER_K_V_RPM);
        }
        
        // 3. Apply Ramping
        long rampDuration = (customRampDurationMs >= 0) ? customRampDurationMs : BjornConstants.Power.SHOOTER_RAMP_DURATION_MS;
        
        if (rampDuration <= 0) {
            commandedRpm = desiredCompensated;
            rampActive = false;
        } else if (rampActive) {
             long dt = nowMs - rampStartTimeMs;
             if (dt >= rampDuration) {
                 commandedRpm = desiredCompensated;
                 rampActive = false;
             } else {
                 // S-Curve
                 double t = (double) dt / rampDuration;
                 double s = t * t * (3.0 - 2.0 * t);
                 commandedRpm = rampStartRpm + (int)((desiredCompensated - rampStartRpm) * s);
             }
        } else {
            // Sustain
            commandedRpm = desiredCompensated;
        }
        
        // 4. Command Motors
        setFlywheelVelocity(commandedRpm);
        
        // 5. Update Ready State
        updateReadyState(nowMs);
        
        // 6. Update LEDs (Visual Readiness)
        updateLeds();
    }
    
    private void updateReadyState(long nowMs) {
        if (targetRpm <= 0) {
            readyLatched = false;
            readyStartMs = 0;
            return;
        }
        
        if (Math.abs(filteredRpm - targetRpm) < READY_TOL_RPM) {
            if (readyStartMs == 0) readyStartMs = nowMs;
            if (!readyLatched && (nowMs - readyStartMs > READY_SETTLE_MS)) {
                readyLatched = true;
                readyAtMs = nowMs;
                long timeToReady = nowMs - targetSetTimeMs;
                // Record metrics on ready? Or on fire?
                // Just keep track.
            }
        } else {
            readyStartMs = 0;
            readyLatched = false; // Unlatch if we drop out? Yes for strict safety.
        }
    }
    
    private void updateLeds() {
        boolean ready = isReady();
        if (ready) {
            if (hardware.led1Green != null) hardware.led1Green.setState(true);
            if (hardware.led1Red != null) hardware.led1Red.setState(false);
            if (hardware.led2Green != null) hardware.led2Green.setState(true);
            if (hardware.led2Red != null) hardware.led2Red.setState(false);
        } else {
             // Red if not ready but active, or just off?
             // If target > 0, show RED. If target == 0, show nothing?
             // TeleOpShooter shows RED when valid but not ready.
             boolean active = targetRpm > 0;
             if (hardware.led1Green != null) hardware.led1Green.setState(false);
             if (hardware.led1Red != null) hardware.led1Red.setState(active);
             if (hardware.led2Green != null) hardware.led2Green.setState(false);
             if (hardware.led2Red != null) hardware.led2Red.setState(active);
        }
    }

    private double readRpm() {
        if (flywheel == null) return 0.0;
        double v1 = flywheel.getVelocity();
        double v2 = (flywheelSecondary != null) ? flywheelSecondary.getVelocity() : v1;
        return ((v1 + v2) / 2.0 / TICKS_PER_REV) * 60.0;
    }

    private void setFlywheelVelocity(int rpm) {
        double tps = (rpm * TICKS_PER_REV) / 60.0;
        if (flywheel != null) flywheel.setVelocity(tps);
        if (flywheelSecondary != null) flywheelSecondary.setVelocity(tps);
    }
}
