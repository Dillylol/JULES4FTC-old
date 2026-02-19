package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * BjornAutoFiringTest
 * 
 * Sequence:
 * 1. Drive Reverse (-Y) using PedroPathing.
 * 2. Ramp Flywheel to 2500 RPM.
 * 3. Swerve Turret to 90 degrees (Heading).
 * 4. Pulse Intake & Grip Servos 3 times (2s ON, 0.5s OFF).
 */
@Autonomous(name = "Bjorn Auto Firing Test", group = "Tests")
public class BjornAutoFiringTest extends OpMode {

    // --- Hardware ---
    private Follower follower;
    private BjornHardware hardware;
    private DcMotorEx turret, intake, wheel, wheel2;
    private IMU imu;

    // --- Turret Control Constants (Copied from BjornCVTurretTest/TeleBlue) ---
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 75.52;
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;
    private static final double TURRET_KP = 0.015;
    private static final double TURRET_KD = 0.004;
    private static final double LIMIT_MIN = BjornConstants.Motors.TURRET_MIN_DEG;
    private static final double LIMIT_MAX = BjornConstants.Motors.TURRET_MAX_DEG;
    private static final double G2_TURRET_POWER = 1.0;
    private static final double MAX_POWER_DELTA = 0.15;

    // --- Test Constants ---
    private static final double TARGET_FLYWHEEL_RPM = 2500.0;
    private static final double TARGET_TURRET_HEADING = 90.0;
    private static final double DRIVE_DISTANCE_INCHES = 20.0; // Drive back 10 inches
    private static final int PULSE_COUNT_TARGET = 3;
    private static final long PULSE_ON_MS = 2000;
    private static final long PULSE_OFF_MS = 500;

    // --- State Machine ---
    private enum State {
        DRIVE_REVERSE,
        RAMP_AND_TURRET,
        PULSE_FIRE,
        DONE
    }

    private State state = State.DRIVE_REVERSE;
    private PathChain reversePath;
    
    // --- State Variables ---
    private double turretAngleDeg = 0.0;
    private int lastEncoderPos = 0;
    private double lastTurretPower = 0.0;
    private double lastTurretError = 0.0;
    private double lastLoopTime = 0;
    private double lastRobotYaw = 0.0;
    
    private int pulseCount = 0;
    private long pulseStartTime = 0;
    private boolean isPulseOn = false;
    private boolean pulsePhaseInit = false;

    @Override
    public void init() {
        // Init Hardware
        hardware = BjornHardware.forAutonomous(hardwareMap);
        turret = hardware.turret;
        intake = hardware.intake;
        wheel = hardware.wheel;
        wheel2 = hardware.wheel2;
        
        // IMU Init
        imu = hardwareMap.get(IMU.class, BjornConstants.Sensors.IMU);
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);
        imu.resetYaw();

        // Pedro Follower
        follower = Constants.createFollower(hardwareMap);
        
        Pose startPose = new Pose(0, 0, 0);
        Pose endPose = new Pose(0, -DRIVE_DISTANCE_INCHES, 0);
        
        follower.setStartingPose(startPose); 

        // Create Path: Reverse 10 inches along Y axis
        reversePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(0)
                .build();

        // Ensure mechanisms off
        setFlywheelRPM(0);
        intake.setPower(0);
        hardware.grip1.setPower(0);
        hardware.grip2.setPower(0);
        
        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();
    }

    @Override
    public void start() {
        lastLoopTime = System.currentTimeMillis() / 1000.0;
        follower.followPath(reversePath);
        state = State.DRIVE_REVERSE;
    }

    @Override
    public void loop() {
        // Time constant
        long nowMs = System.currentTimeMillis();
        double nowSec = nowMs / 1000.0;
        double dt = (lastLoopTime > 0) ? (nowSec - lastLoopTime) : 0.02;
        lastLoopTime = nowSec;

        follower.update();
        updateTurret(dt); // Always keep turret active/corrected

        switch (state) {
            case DRIVE_REVERSE:
                if (!follower.isBusy()) {
                    // Path complete
                    state = State.RAMP_AND_TURRET;
                }
                break;

            case RAMP_AND_TURRET:
                // 1. Ramp Flywheel
                setFlywheelRPM(TARGET_FLYWHEEL_RPM);

                // 2. Turret Target is set in updateTurret via constant, check error
                double error = TARGET_TURRET_HEADING - turretAngleDeg;
                
                // Wait for turret to be close (e.g. within 5 degrees) AND Flywheel up to speed?
                // User said "when the turret is ready, turn on the intake..."
                // Simplified: just wait for turret angle.
                if (Math.abs(error) < 5.0) {
                     // Check flywheel speed too? Or just assume it ramps fast enough during swerve.
                     // User said "ramp up, when turret is ready".
                     state = State.PULSE_FIRE;
                     pulseCount = 0;
                     pulsePhaseInit = true; // Flag to start first pulse
                }
                break;

            case PULSE_FIRE:
                // Maintain Flywheel & Turret (handled by loop top & updateTurret)

                if (pulsePhaseInit) {
                    pulseStartTime = nowMs;
                    isPulseOn = true;
                    setIntakeGrips(true);
                    pulsePhaseInit = false;
                }

                long elapsed = nowMs - pulseStartTime;

                if (isPulseOn) {
                    if (elapsed >= PULSE_ON_MS) {
                        // End of ON pulse
                        setIntakeGrips(false);
                        isPulseOn = false;
                        pulseStartTime = nowMs; // Reset for OFF phase
                    }
                } else {
                    if (elapsed >= PULSE_OFF_MS) {
                        // End of OFF pulse
                        pulseCount++;
                        if (pulseCount >= PULSE_COUNT_TARGET) {
                            state = State.DONE;
                        } else {
                            // Start next ON pulse
                            isPulseOn = true;
                            setIntakeGrips(true);
                            pulseStartTime = nowMs;
                        }
                    }
                }
                break;

            case DONE:
                setFlywheelRPM(0);
                intake.setPower(0);
                hardware.grip1.setPower(0);
                hardware.grip2.setPower(0);
                follower.breakFollowing();
                break;
        }
        
        telemetry.addData("State", state);
        telemetry.addData("Turret Angle", turretAngleDeg);
        telemetry.addData("Pulse Count", pulseCount);
        telemetry.update();
    }

    // --- Helpers ---

    private void setIntakeGrips(boolean on) {
        double pwr = on ? 1.0 : 0.0;
        // Intake needs specific power? constant says .70 usually, user said "turn on".
        // BjornConstants probably has a value. Using 0.8 for now or constant if available.
        // checking constants found in BjornAutoBLUE: INTAKE_POWER = .70
        double intakePwr = on ? 0.7 : 0.0;
        // Intake Power - Hardware configures direction
        intake.setPower(intakePwr); 
        
        // Grip Servos (CRServo)
        // Check direction in Hardware.
       hardware.grip1.setPower(pwr);
       hardware.grip2.setPower(pwr);
    }

    private void setFlywheelRPM(double rpm) {
        // Simple feedforward: RPM / MaxRPM
        // BjornAutoBLUE uses: clamp(rpm / WHEEL_MAX_RPM, 0, 1)
        double maxRpm = 4000.0; // From AutoBlue
        double pwr = Range.clip(rpm / maxRpm, 0.0, 1.0);
        wheel.setPower(pwr);
        if (wheel2 != null) wheel2.setPower(pwr);
    }

    private void updateTurret(double dt) {
        // Encoder Update
        int currentPos = turret.getCurrentPosition();
        int deltaTicks = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;
        turretAngleDeg += (deltaTicks / TURRET_TICKS_PER_DEGREE) * BjornConstants.Motors.TURRET_ENCODER_DIRECTION;

        // PD Control
        // Fetch robot Heading (Yaw)
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double robotRate = (dt > 0) ? (robotYaw - lastRobotYaw) / dt : 0.0;
        lastRobotYaw = robotYaw;

        // Desired Angle Logic (Matches BjornTeleRed.java)
        // Note: BjornTeleRed uses 'targetFieldHeading + robotYaw'. 
        // This implies Turret Positive is CLOCKWISE (Right) and Robot Positive is CCW (Left),
        // or a similar convention where they sum to stabilize.
        double desiredTurretAngle = TARGET_TURRET_HEADING + robotYaw;
        
        // Clamp Target
        desiredTurretAngle = Range.clip(desiredTurretAngle, LIMIT_MIN, LIMIT_MAX);
        
        double error = desiredTurretAngle - turretAngleDeg;
        
        if (Math.abs(error) < 2.0) {
            error = 0; // Deadband
        }

        double derivative = (dt > 0) ? (error - lastTurretError) / dt : 0.0;
        double pid = (error * TURRET_KP) + (derivative * TURRET_KD);
        
        // Feedforward from TeleOp
        double ff = robotRate * 0.0055; // ROBOT_ROTATION_FF_GAIN
        
        double pwr = pid + ff;

        // Limits
        if (turretAngleDeg < LIMIT_MIN && pwr < 0) pwr = 0;
        if (turretAngleDeg > LIMIT_MAX && pwr > 0) pwr = 0;

        // Clamp and Slew
        if (Double.isNaN(pwr)) pwr = 0;
        pwr = Range.clip(pwr, -G2_TURRET_POWER, G2_TURRET_POWER);
        
        double powerDelta = Range.clip(pwr - lastTurretPower, -MAX_POWER_DELTA, MAX_POWER_DELTA);
        double slewPower = lastTurretPower + powerDelta;
        
        // Force Field Override (TeleOp Feature)
        if (turretAngleDeg < LIMIT_MIN) {
            slewPower = 0.6; // Push Right
        } else if (turretAngleDeg > LIMIT_MAX) {
            slewPower = -0.6; // Push Left
        }

        lastTurretPower = slewPower;
        turret.setPower(slewPower * BjornConstants.Motors.TURRET_POWER_DIRECTION);
    }
}
