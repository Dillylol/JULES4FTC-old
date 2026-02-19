package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.configurables.BjornTurretTunerConfig;
import org.firstinspires.ftc.teamcode.configurables.TurretConfigurables;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Method;

/**
 * Self-Contained Turret Tuner
 * 
 * Standalone OpMode for Testing and Tuning the Turret PID + FeedForward.
 * - Initializes its own Hardware (Turret + IMU) to ensure isolation.
 * - Implements Auto-Tuning (Step Response).
 * - Implements Live Verification (Square Wave) using Config Panels
 * - Implements Counter-Rotation Testing.
 * 
 * NOTE: TEMPORARILY DISABLED - Incompatible with new TurretConfigurables API
 * Need to update to use new field names (no feedForward, no kI)
 */
@Configurable
// DISABLED: @TeleOp(name = "Bjorn Turret Self-Contained Tuner", group = "Test")
public class BjornTurretSelfContainedTuner extends OpMode {

    // --- Hardware ---
    private DcMotorEx turret;
    private IMU imu;

    // --- Local Constants (Mirrored from BjornConstants/Hardware for Isolation) ---
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 48.0;
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;
    
    // Limits
    private static final double HARD_MIN = -10.0;
    private static final double HARD_MAX = 170.0;
    private static final double CENTER = 90.0;
    private static final double STOP_ANGLE = 135.0; 
    
    // Auto-Tune Params
    private static final double STEP_POWER = 0.65;
    private double t0 = 0;
    
    // --- State Machine ---
    private enum State {
        IDLE,
        CENTERING,
        STEP_TEST,
        CALCULATED,
        LIVE_VERIFY
    }
    private State currentState = State.IDLE;

    // --- Data Collection ---
    private double maxVelocity = 0;
    private double calcKp = 0, calcKd = 0, calcKv = 0;
    
    // --- Runtime ---
    private double lastLoopTime = 0;
    private int lastEncoderPos = 0;
    private double turretAngleDeg = 0;
    private double lastError = 0;
    private double lastIntegral = 0;
    private double timer = 0;
    private double lastRobotYaw = 0;
    private double zeroYawOffset = 0;

    // --- Counter Rotation Control ---
    private boolean counterRotationEnabled = true;
    private static final double ROBOT_ROTATION_FF_GAIN = 0.0055;

    // --- Inputs ---
    private boolean aPrev = false;
    private boolean xPrev = false;
    private boolean yPrev = false;
    private boolean rbPrev = false;

    // --- Panels Integration ---
    private Object panelsMgr;
    private Method asCombinedMethod;
    private TelemetryManager telemetryManager;

    @Override
    public void init() {
        // --- 1. Init Turret ---
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(DcMotor.Direction.REVERSE); // User confirmed REVERSE
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Raw power control

        // --- 2. Init IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // --- 3. Panels Init ---
        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        try {
            panelsMgr = PanelsGamepad.INSTANCE.getFirstManager();
            if (panelsMgr != null) {
                asCombinedMethod = panelsMgr.getClass().getMethod("asCombinedFTCGamepad", Gamepad.class);
            }
        } catch (Exception e) {
            telemetry.addData("Panels Error", e.getMessage());
        }

        telemetryManager.debug("=== Self-Contained Turret Tuner ===");
        telemetryManager.debug("[A] Start Auto-Tune");
        telemetryManager.debug("[X] Toggle Live PID (Hold Mode)");
        telemetryManager.debug("[RB] Toggle Oscillation (Target Shift)");
        telemetryManager.debug("[Y] Toggle Counter-Rotation (Disturbance Rejection)");
        telemetryManager.debug("Manually rotate the test rig for CR test");
        telemetryManager.update(telemetry);
    }

    @Override
    public void loop() {
        // Time Delta
        long nowMs = System.currentTimeMillis();
        double now = nowMs / 1000.0;
        double dt = (lastLoopTime > 0) ? (now - lastLoopTime) : 0.02;
        lastLoopTime = now;

        // --- Read Sensors ---
        // 1. Turret Angle
        int currentPos = turret.getCurrentPosition();
        int deltaTicks = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;
        // Encoder Direction is normally -1.0 in Bjorn constants. 
        // Logic: If Turret Motor is REVERSE for Positive Power => Right, 
        // does Encoder count Up or Down? 
        // Let's stick to the multiplier used in BjornHardware default if possible, 
        // but user asked to define LOCALLY.
        // Assuming standard behavior: +Power -> +Counts.
        // If we want +Angle to be Right, and +Power is Right...
        // We'll trust the hardware direction set above.
        turretAngleDeg += (deltaTicks / TURRET_TICKS_PER_DEGREE); 
        
        double velocity = (deltaTicks / TURRET_TICKS_PER_DEGREE) / dt;

        // 2. IMU Yaw & Rate
        double robotYaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        // Use Native Gyro Rate for instantaneous response (Z-axis rotation)
        double robotRate = -imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        lastRobotYaw = robotYaw;

        // --- Safe Limits ---
        if (turretAngleDeg < HARD_MIN || turretAngleDeg > HARD_MAX) {
            if (currentState != State.IDLE && currentState != State.CALCULATED) {
                turret.setPower(0);
                currentState = State.IDLE;
                telemetry.addData("STATUS", "LIMIT HIT! RESET.");
            }
        }

        // --- Panels Gamepad ---
        Gamepad g1 = getCombinedGamepad(gamepad1);

        // --- Inputs ---
        // E-Stop
        if (g1.b) {
            turret.setPower(0);
            currentState = State.IDLE;
        }

        // [A] Auto-Tune
        if (g1.a && !aPrev && currentState == State.IDLE) {
            currentState = State.CENTERING;
            resetData();
        }
        aPrev = g1.a;

        // [X] Live Verify (Enable/Disable PID Loop)
        if (g1.x && !xPrev) {
            if (currentState == State.LIVE_VERIFY) {
                currentState = State.IDLE;
                BjornTurretTunerConfig.runTest = false;
            } else {
                currentState = State.LIVE_VERIFY;
                BjornTurretTunerConfig.runTest = false; // Default to Steady Hold
                timer = 0;
            }
        }
        xPrev = g1.x;

        // [Right Bumper] Toggle Oscillation (Target Movement)
        if (g1.right_bumper && !rbPrev) {
             BjornTurretTunerConfig.runTest = !BjornTurretTunerConfig.runTest;
        }
        rbPrev = g1.right_bumper;

        // [Y] Toggle Counter-Rotation
        if (g1.y && !yPrev) {
            counterRotationEnabled = !counterRotationEnabled;
            if (counterRotationEnabled) {
                zeroYawOffset = robotYaw; // Capture current yaw as "Zero" for world lock
            }
        }
        yPrev = g1.y;


        // --- State Machine ---
        switch (currentState) {
            case IDLE:
                turret.setPower(0);
                break;

            case CENTERING:
                if (moveSafelyTo(CENTER, dt)) {
                    turret.setPower(0);
                    t0 = now;
                    currentState = State.STEP_TEST;
                }
                break;

            case STEP_TEST:
                turret.setPower(STEP_POWER);
                double absVel = Math.abs(velocity);
                if (absVel > maxVelocity) maxVelocity = absVel;
                if (turretAngleDeg >= STOP_ANGLE) {
                    turret.setPower(0);
                    calculateResults(STEP_POWER, maxVelocity);
                    currentState = State.CALCULATED;
                }
                break;

            case CALCULATED:
                turret.setPower(0);
                telemetryManager.debug("=== CALCULATION COMPLETE ===");
                telemetryManager.debug(String.format("Max Vel: %.1f deg/s", maxVelocity));
                telemetryManager.debug(String.format("Rec. kP: %.5f", calcKp));
                telemetryManager.debug(String.format("Rec. kD: %.5f", calcKd));
                telemetryManager.debug(String.format("Rec. FF: %.5f", calcKv));
                break;

            case LIVE_VERIFY:
                runLiveVerify(dt, robotRate, robotYaw);
                break;
        }

        // --- Manual Drive Control (For Disturbance Testing) ---
        // --- Manual Drive Control (For Disturbance Testing) ---
        // updateDrive(); // Removed for Test Article

        // --- Telemetry ---
        telemetryManager.debug("State: " + currentState);
        telemetryManager.debug(String.format("Angle: %.1f", turretAngleDeg));
        telemetryManager.debug("Oscillation [RB]: " + BjornTurretTunerConfig.runTest);
        telemetryManager.debug("CR Active [Y]: " + counterRotationEnabled);
        telemetryManager.debug(String.format("Robot Yaw: %.1f", robotYaw));
        
        telemetryManager.update(telemetry);
    }
    
    private Gamepad getCombinedGamepad(Gamepad original) {
        if (panelsMgr != null && asCombinedMethod != null) {
            try {
                return (Gamepad) asCombinedMethod.invoke(panelsMgr, original);
            } catch (Exception ignored) { }
        }
        return original;
    }

    private void resetData() {
        maxVelocity = 0;
        lastError = 0;
        lastIntegral = 0;
    }

    private boolean moveSafelyTo(double target, double dt) {
        double error = target - turretAngleDeg;
        double derivative = (error - lastError) / dt;
        lastError = error;

        // Use safe tuning values
        double kp = 0.00379;
        double kd = 0.00011;
        double kStatic = 0.5;

        double pid = (error * kp) + (derivative * kd);
        double ff = Math.signum(error) * kStatic;
        double power = Range.clip(pid + ff, -0.7, 0.7);

        turret.setPower(power);
        // Relaxed threshold
        return Math.abs(error) < 3.0;
    }

    private void calculateResults(double inputPct, double maxVelDegS) {
        double K = maxVelDegS / inputPct;
        calcKv = inputPct / maxVelDegS;
        double Tau = 0.1;
        double DeadTime = 0.04;
        double rawKp = (1.0 / K) * (Tau / DeadTime);
        calcKp = rawKp * 0.6;
        calcKd = calcKp * Tau * 0.3;
    }

    private void runLiveVerify(double dt, double robotRate, double robotYaw) {
        timer += dt;
        
        double target;
        if (BjornTurretTunerConfig.runTest) {
             // Oscillate
             double period = Math.max(0.1, BjornTurretTunerConfig.periodSec);
             double phase = timer % period;
             target = (phase < (period / 2.0)) ? BjornTurretTunerConfig.targetHigh : BjornTurretTunerConfig.targetLow;
        } else {
             // Hold Steady
             target = BjornTurretTunerConfig.manualTarget;
        }

        // World Lock Compensation
        double compensatedTarget = target;
        if (counterRotationEnabled) {
             // If Robot Turns Left (+Yaw), Target Relative Angle must Decrease.
             // Target_Rel = Target_World - (Robot_Current - Robot_Zero)
             compensatedTarget = target - (robotYaw - zeroYawOffset);
        }

        double error = compensatedTarget - turretAngleDeg;

        // 1. Deadband Logic (User Request)
        double staticFrictionFF = 0.0;
        if (Math.abs(error) < TurretConfigurables.deadband) {
            error = 0.0;
        } else {
            staticFrictionFF = 0.0; // Disabled - TurretConfigurables.feedForward removed
        }

        double derivative = (error - lastError) / dt;
        lastIntegral += (error * dt);
        lastIntegral = Range.clip(lastIntegral, -50, 50);
        lastError = error;

        // 2. PID
        double pid = (error * TurretConfigurables.kP) +
                     (lastIntegral * 0.0) + // kI removed from TurretConfigurables
                     (derivative * TurretConfigurables.kD);

        // 3. Counter-Rotation Feedforward
        double rotationFF = 0.0;
        if (counterRotationEnabled) {
            rotationFF = -robotRate * BjornTurretTunerConfig.counterRotationGain;
        }

        double totalPower = pid + rotationFF + staticFrictionFF;
        totalPower = Range.clip(totalPower, -1.0, 1.0);
        
        turret.setPower(totalPower);

        telemetryManager.debug("Tgt: " + target);
        telemetryManager.debug("WorldTgt: " + compensatedTarget);
        telemetryManager.debug("Err: " + error);
        telemetryManager.debug("RotFF: " + rotationFF);
    }

}
