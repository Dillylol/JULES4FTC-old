package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.configurables.BjornTurretTunerConfig;
import org.firstinspires.ftc.teamcode.configurables.TurretConfigurables;

/**
 * JULES Turret Auto-Tuner v11 - Step Response & Live Verify
 * 
 * Functions:
 * 1. Auto-Tune (A Button): Runs Step Response to finding Kp, Kd, Kv (FeedForward).
 * 2. Live Verify (X Button): Runs Square Wave using current Config settings.
 * 
 * NOTE: TEMPORARILY DISABLED - Incompatible with new TurretConfigurables API
 * Need to update to use new field names (no feedForward, no kI)
 */
// DISABLED: @TeleOp(name = "Bjorn Turret Auto Tuner", group = "Test")
public class BjornTurretAutoTuner extends BjornTeleBase {

    private DcMotorEx turret;
    
    // --- Limits ---
    public static double HARD_MIN = -10.0;
    public static double HARD_MAX = 170.0;
    private static final double CENTER = 90.0;
    private static final double STOP_ANGLE = 135.0; // Stop step test here
    private static final double STEP_POWER = 0.65;  // INCREASED for Friction (min 0.55)
    
    private enum State {
        IDLE,
        CENTERING, // Move to 90 safely
        RESETTING, 
        STEP_TEST, // Apply constant voltage, measure curve
        CALCULATED, // Show results
        LIVE_VERIFY // Square wave mode
    }
    private State currentState = State.IDLE;
    
    // --- Data Collection ---
    private double t0 = 0;
    private double maxVelocity = 0;
    private double calcKp = 0, calcKd = 0, calcKv = 0;
    
    // --- Runtime ---
    private double lastLoopTime = 0;
    private int lastEncoderPos = 0;
    private double turretAngleDeg = 0;
    private double lastError = 0;
    private double lastIntegral = 0;
    private double timer = 0;
    
    // Input Debounce
    private boolean aPrev = false;
    private boolean xPrev = false;
    
    @Override
    public void init() {
        initSubsystems(); 
        this.turret = hardware.turret; // Use BjornHardware instance
        
        // Override for Tuner (User reported backwards)
        turret.setDirection(DcMotor.Direction.REVERSE);
        
        // Tuner needs raw control
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        telemetry.addLine("=== Bjorn Turret Tuner ===");
        telemetry.addLine("[A] -> START AUTO-TUNE (Calculates PIDF)");
        telemetry.addLine("[X] -> TOGGLE LIVE VERIFY (Uses Config Panel)");
        telemetry.update();
    }
    
    @Override
    protected String getAllianceName() { return "TEST"; }
    @Override
    protected int getGoalTagId() { return 0; }

    @Override
    public void loop() {
        // E-STOP
        if (gamepad1.b || gamepad2.b) {
            turret.setPower(0);
            currentState = State.IDLE;
            BjornTurretTunerConfig.runTest = false;
        }
        
        // Auto-Tune Start
        if (gamepad1.a && !aPrev && currentState == State.IDLE) {
             currentState = State.CENTERING;
             resetData();
        }
        aPrev = gamepad1.a;

        // Live Verify Toggle
        if (gamepad1.x && !xPrev) {
            if (currentState == State.LIVE_VERIFY) {
                currentState = State.IDLE;
                BjornTurretTunerConfig.runTest = false;
            } else {
                currentState = State.LIVE_VERIFY;
                BjornTurretTunerConfig.runTest = true;
                timer = 0;
            }
        }
        xPrev = gamepad1.x;

        long nowMs = System.currentTimeMillis();
        double now = nowMs / 1000.0;
        double dt = (lastLoopTime > 0) ? (now - lastLoopTime) : 0.02;
        lastLoopTime = now;
        
        // --- Read Hardware ---
        int currentPos = turret.getCurrentPosition();
        int deltaTicks = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;
        
        // Angle Calculation
        turretAngleDeg += (deltaTicks / BjornHardware.TURRET_TICKS_PER_DEGREE); 
        double velocity = (deltaTicks / BjornHardware.TURRET_TICKS_PER_DEGREE) / dt;

        // Safety Limit
        if (turretAngleDeg < HARD_MIN || turretAngleDeg > HARD_MAX) {
             // Exception: During Centering we might be near limits, be careful
             if (currentState != State.IDLE && currentState != State.CALCULATED) {
                 turret.setPower(0);
                 currentState = State.IDLE;
                 telemetry.addData("STATUS", "LIMIT HIT! RESET.");
             }
        }

        switch (currentState) {
            case IDLE:
                turret.setPower(0);
                break;
                
            case CENTERING:
                // Move safely to 90
                if (moveSafelyTo(CENTER, dt)) {
                    turret.setPower(0);
                    // Wait for settle?
                    t0 = now;
                    currentState = State.STEP_TEST;
                }
                break;
                
            case STEP_TEST:
                // Apply Step
                turret.setPower(STEP_POWER); 
                
                // Capture Max Velocity
                double absVel = Math.abs(velocity);
                if (absVel > maxVelocity) maxVelocity = absVel;
                
                // End Condition
                if (turretAngleDeg >= STOP_ANGLE) {
                    turret.setPower(0);
                    calculateResults(STEP_POWER, maxVelocity);
                    currentState = State.CALCULATED;
                }
                break;
                
            case CALCULATED:
                turret.setPower(0);
                telemetry.addLine("=== CALCULATION COMPLETE ===");
                telemetry.addData("Max Vel", "%.1f deg/s", maxVelocity);
                telemetry.addData("Rec. kP", "%.5f", calcKp);
                telemetry.addData("Rec. kD", "%.5f", calcKd);
                telemetry.addData("Rec. FF", "%.5f", calcKv);
                telemetry.addLine("Update 'TurretConfigurables' with these values!");
                telemetry.addLine("Press X to Verify with current Config");
                break;
                
            case LIVE_VERIFY:
                runLiveVerify(dt);
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.addData("Angle", "%.1f", turretAngleDeg);
        telemetry.update();
    }
    
    private void resetData() {
        maxVelocity = 0;
        lastError = 0;
        lastIntegral = 0;
    }
    
    // Simple Centering P-Controller
    private boolean moveSafelyTo(double target, double dt) {
        double error = target - turretAngleDeg;
        double derivative = (error - lastError) / dt;
        lastError = error;
        
        double kp = 0.00379; 
        double kd = 0.00011;
        double kStatic = 0.5;
        
        double pid = (error * kp) + (derivative * kd);
        double ff = Math.signum(error) * kStatic;
        
        double power = pid + ff;
        power = Range.clip(power, -0.7, 0.7); // Increased limit to allow movement
        
        turret.setPower(power);
        
        return Math.abs(error) < 3.0; // Settled enough for Step Test
    }
    
    private void calculateResults(double inputPct, double maxVelDegS) {
        // 1. Process Gain K = Speed / %
        double K = maxVelDegS / inputPct;
        
        // 2. FeedForward (Kv) approx
        calcKv = inputPct / maxVelDegS; 
        
        // 3. Time Constant (Tau) Estimate
        double Tau = 0.1; 
        double DeadTime = 0.04; 
        
        // 4. Cohen-Coon Simplified
        double rawKp = (1.0 / K) * (Tau / DeadTime);
        calcKp = rawKp * 0.6;
        calcKd = calcKp * Tau * 0.3; 
    }
    
    private void runLiveVerify(double dt) {
         timer += dt;
        double period = Math.max(0.1, BjornTurretTunerConfig.periodSec);
        double phase = timer % period; 
        double target = (phase < (period / 2.0)) ? BjornTurretTunerConfig.targetHigh : BjornTurretTunerConfig.targetLow;
        
        double error = target - turretAngleDeg;
        
        // Apply Deadband to prevent oscillation from high Static Friction compensation
        double ffTerm = 0.0;
        if (Math.abs(error) < TurretConfigurables.deadband) {
            error = 0.0; // Assume settled
        } else {
            ffTerm = 0.0; // Disabled - TurretConfigurables.feedForward removed
        }

        double derivative = (error - lastError) / dt;
        lastIntegral += (error * dt);
        lastIntegral = Range.clip(lastIntegral, -50, 50);
        lastError = error;
        
        double pid = (error * TurretConfigurables.kP) + 
                     (lastIntegral * 0.0) + // kI removed from TurretConfigurables
                     (derivative * TurretConfigurables.kD) +
                     ffTerm;
        
        double power = Range.clip(pid, -1.0, 1.0);
        turret.setPower(power);
        
        telemetry.addData("Tgt", target);
        telemetry.addData("Err", error);
    }
}
