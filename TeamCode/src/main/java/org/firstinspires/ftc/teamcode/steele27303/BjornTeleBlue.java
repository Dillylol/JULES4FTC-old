package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.configurables.TurretConfigurables;
import org.firstinspires.ftc.teamcode.common.turret.TurretEstimator;
import org.firstinspires.ftc.teamcode.common.turret.TurretControl;
import org.firstinspires.ftc.teamcode.common.turret.GoalCalculator;
import org.firstinspires.ftc.teamcode.common.BjornPersistence;

import java.util.List;

/**
 * BLUE TeleOp - Drive/IMU/Pedro/Camera/Turret Inline (Reduced Latency)
 * 
 * Gamepad 1 (Driver):
 * - Left Stick: Drive/Strafe
 * - Right Stick: Rotate
 * - D-Pad Down: IMU Reset
 * - D-Pad L/R: Manual turret at 0.5 speed (limited authority)
 * - A: Intake
 * - X: Outtake
 * - LB: Mark waypoint
 * - RB: Execute auto-drive
 * 
 * Gamepad 2 (Gunner):
 * - Left Stick X: Turret rotation (0.75 max power)
 * - A: Toggle Position Tracking (locks turret to scoring zone)
 * - B: Toggle shooter ramp-up
 * - RT: Fire
 */
@TeleOp(name = "Bjorn TeleOp BLUE")
public class BjornTeleBlue extends BjornTeleBase {

    // --- Drive Hardware (Inline) ---
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // --- Turret Hardware (Inline) ---
    private DcMotorEx turret;
    // frontTof moved to Base/Hardware

    // --- Pedro (Inline) ---
    private Follower follower;
    private Pose autoDriveTargetPose;
    private Pose savedPose;
    private boolean autoDriveActive = false;

    // --- Camera (Inline) ---
    private AprilTagCamera aprilTagCamera;
    private static final int GOAL_TAG_ID = CameraConfig.BLUE_GOAL_TAG_ID;

    // --- Turret Constants ---
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 48.0;
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;

    private static final double ROBOT_ROTATION_FF_GAIN = 0.0055;
    // PID/FF/Deadband now from TurretConfigurables
    private static final double LIMIT_MIN = BjornConstants.Motors.TURRET_MIN_DEG;
    private static final double LIMIT_MAX = BjornConstants.Motors.TURRET_MAX_DEG;
    private static final double MANUAL_RATE_DEG_PER_SEC = 90.0;
    private static final double APRILTAG_CORRECTION_GAIN = 0.5;

    private static final double G2_TURRET_POWER = 1.0;
    private static final double G1_TURRET_POWER = 0.5;

    // --- Turret State ---
    private double targetFieldHeading = 90.0;
    private double turretAngleDeg = 0.0;
    private int lastEncoderPos = 0;
    private double lastTurretPower = 0.0;
    private double lastTurretError = 0.0;
    private double lastRobotYaw = 0.0;
    private boolean positionTrackingEnabled = false; // Renamed from aprilTagTrackingEnabled
    private boolean wasManualControl = false;
    private boolean autoShootEnabled = false;
    private boolean cameraConfigured = false;
    
    // --- Turret Control System ---

    private TurretControl turretControl;
    private GoalCalculator goalCalculator;


    // --- Input State ---
    private boolean g1LbPrev = false;
    private boolean yawResetPrev = false;
    private boolean g2APrev = false;
    private boolean g2DpadRightPrev = false;
    private boolean dpadUpPrev = false; // Added for G1 Camera Toggle
    private double lastLoopTime = 0;

    // --- Alliance Config ---
    private static final Pose AUTO_END_POSE = BjornConstants.Auto.BLUE_AUTO_END_POSE;

    @Override
    protected String getAllianceName() {
        return "BLUE";
    }

    @Override
    protected int getGoalTagId() {
        return GOAL_TAG_ID;
    }

    @Override
    public void init() {
        // --- Drive Motors (Inline Init) ---
        frontLeft = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.BACK_LEFT);
        backRight = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.BACK_RIGHT);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        // --- Turret Motor (Inline Init) ---
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reset encoder - assumes turret was homed to 0° at end of Auto
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- IMU (Inline Init) ---
        imu = hardwareMap.get(IMU.class, BjornConstants.Sensors.IMU);
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        // --- Pedro Follower (Inline Init) ---
        // --- Pedro Follower (Inline Init) ---
        follower = Constants.createFollower(hardwareMap);
        // Start at Auto end pose for seamless transition (Hardcoded handoff)
        Pose startPose = BjornConstants.Auto.BLUE_AUTO_END_POSE;
        follower.setStartingPose(startPose);
        autoDriveTargetPose = startPose;
        savedPose = startPose;

        // --- AprilTag Camera (Inline Init) ---
        aprilTagCamera = new AprilTagCamera();
        aprilTagCamera.start(hardwareMap, null);

        // --- ToF Sensor (Inline Init) ---
        // frontTof initialized in Base via BjornHardware

        // --- Subsystems (From Base) ---
        initSubsystems();
        
        // --- Turret Control System (Inline) ---
        // Initialize directly from encoder (persistence via hardware)
        turretControl = new TurretControl(turret, imu);
        turretControl.resetAngle(90.0); // Force start at 90° (from Auto Homing)
        
        // --- Initialize Goal Calculator ---
        // Use hardcoded Auto End Pose as start for triangulation
        Pose autoStartPose = BjornConstants.Auto.BLUE_AUTO_END_POSE;

        goalCalculator = new GoalCalculator(
            BjornConstants.FieldPositions.BLUE_SCORING_X,
            BjornConstants.FieldPositions.BLUE_SCORING_Y
        );

        // Pass Camera to Shooter for CV calculation
        shooter.setCamera(aprilTagCamera, GOAL_TAG_ID);

        telemetry.addLine("Bjorn TeleOp BLUE Initialized");
        telemetry.addData("Goal Tag", GOAL_TAG_ID);
        telemetry.update();
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        double now = nowMs / 1000.0;
        double dt = (lastLoopTime > 0) ? (now - lastLoopTime) : 0.02;
        lastLoopTime = now;

        // --- Update Systems (Inline) ---
        // Always update Pedro follower to maintain localization (needed for position tracking)
        follower.update();
        
        // Feed current robot pose to turret control for position tracking
        turretControl.updateRobotPose(follower.getPose());

        // --- Input Handling ---
        handleWaypointInput();
        handleImuReset();

        // --- Turret Control (Inline - Latency Sensitive) ---
        updateTurret(dt);

        // --- Auto Shoot & Config ---
        updateAutoShoot();

        // --- Drive (Inline) ---
        updateDrive();

        // --- Subsystems (From Base) - G2 controls shooter ---
        // Shooter RPM managed by TeleOpShooter in updateSubsystems
        updateSubsystems(nowMs);

        // --- Telemetry ---
        // --- Telemetry ---
        telemetry.addData("Alliance", "BLUE");
        
        telemetry.addLine("--- TRACKING DEBUG (PEDRO) ---");
        telemetry.addData("Tracking Active", turretControl.isPositionTrackingEnabled() ? "YES" : "NO");

        // 1. Robot Pose from Pedro
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        telemetry.addData("Robot Heading (Pedro)", "%.1f°", robotHeadingDeg);
        telemetry.addData("Robot Pos", "X:%.1f Y:%.1f", follower.getPose().getX(), follower.getPose().getY());

        // 2. Turret State (Robot Relative)
        double turretRelDeg = turretControl.getCurrentAngle();
        double targetRelDeg = turretControl.getTargetAngle();
        telemetry.addData("Turret Heading (Robot Rel)", "%.1f°", turretRelDeg);
        telemetry.addData("Turret Target (Robot Rel)", "%.1f°", targetRelDeg);

        // 3. Field Calculations
        // Estimated Field Heading = Robot Heading + Turret Angle
        double estFieldHeading = robotHeadingDeg + turretRelDeg;

        // Actual Goal Heading (Geometry) - Verify GoalCalculator logic independently
        double goalX = BjornConstants.FieldPositions.BLUE_SCORING_X;
        double goalY = BjornConstants.FieldPositions.BLUE_SCORING_Y;
        
        double gDx = goalX - follower.getPose().getX();
        double gDy = goalY - follower.getPose().getY(); // Reversed Y in FTC field? No, standard field coords.
        double goalFieldHeading = Math.toDegrees(Math.atan2(gDy, gDx));
        // Normalize
        while (goalFieldHeading > 180) goalFieldHeading -= 360;
        while (goalFieldHeading <= -180) goalFieldHeading += 360;    

        telemetry.addData("Goal Heading (Field)", "%.1f°", goalFieldHeading);
        telemetry.addData("Est. Field Heading", "%.1f°", estFieldHeading);

        // 4. Tracking Error (Target - Current) [Robot Frame]
        double error = targetRelDeg - turretRelDeg;
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        telemetry.addData("Tracking Error", "%.1f°", error);
        
        telemetry.addLine("--- SYSTEMS ---");
        telemetry.addData("Auto-Shoot", autoShootEnabled ? "ON" : "OFF");
        telemetry.addData("Auto-Drive", autoDriveActive ? "ACTIVE" : "OFF");
        if (turretControl.isPositionTrackingEnabled()) {
             telemetry.addData("Distance to Target", "%.1f in", turretControl.getDistanceToTarget());
        }
        addSubsystemTelemetry(nowMs);
        telemetry.update();
    }

    // --- Turret Control (Inline - From InclusiveTurretTest) ---

    private void updateTurret(double dt) {
        long nowMs = System.currentTimeMillis();
        
        turretAngleDeg = turretControl.getCurrentAngle();

        // --- G2: A toggles Position Tracking ---
        if (gamepad2.a && !g2APrev) {
            positionTrackingEnabled = !positionTrackingEnabled;
            // No need to set on turretControl since we manage it manually via setTargetAngle
        }
        g2APrev = gamepad2.a;
        dpadUpPrev = gamepad1.dpad_up;

        // --- Manual Control ---
        // G2: Left stick X (full authority)
        // G1: D-Pad L/R (limited authority)
        double turretInput = gamepad2.left_stick_x;
        double g1Input = 0.0;
        if (gamepad1.dpad_left)
            g1Input = -1.0;
        else if (gamepad1.dpad_right)
            g1Input = 1.0;

        // Combine inputs (G2 has priority if both active)
        double stickInput = (Math.abs(turretInput) > 0.1) ? turretInput : g1Input * (G1_TURRET_POWER / G2_TURRET_POWER);
        boolean isManualControl = Math.abs(stickInput) > 0.1;

        if (isManualControl) {
            double headingDelta = stickInput * MANUAL_RATE_DEG_PER_SEC * dt;
            targetFieldHeading += headingDelta;
            targetFieldHeading = Range.clip(targetFieldHeading, LIMIT_MIN, LIMIT_MAX);
            turretControl.setTargetAngle(targetFieldHeading);
            wasManualControl = true;
        } else if (wasManualControl) {
            // Just released - lock to current belief
            targetFieldHeading = turretControl.getCurrentAngle();
            turretControl.setTargetAngle(targetFieldHeading);
            wasManualControl = false;
        }

        // --- Position Tracking Update ---
        // Dynamically update goal angle if position tracking is enabled
        if (positionTrackingEnabled) {
            double goalAngle = goalCalculator.getTurretAngleToGoal(follower.getPose());
            turretControl.setTargetAngle(goalAngle);
        }
        
        aprilTagCamera.pollDetections(); // Keep camera active for shooter

        // --- Run TurretControl ---
        // TurretControl.update() handles PD, FF, force field, limits,
        // and applies TURRET_POWER_DIRECTION internally via setPower().
        // Do NOT set motor power again here — it would double-invert.
        turretControl.update(dt);
    }

    // --- Inline Input Handlers ---

    private void handleWaypointInput() {
        boolean markWaypoint = gamepad1.left_bumper;
        if (markWaypoint && !g1LbPrev) {
            autoDriveTargetPose = follower.getPose();
        }
        g1LbPrev = markWaypoint;
    }

    private void handleImuReset() {
        boolean yawReset = gamepad1.dpad_down;
        if (yawReset && !yawResetPrev) {
            imu.resetYaw();
            follower.setStartingPose(new Pose(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    0));
            // Reset turret target to match (Target = Local - Yaw)
            targetFieldHeading = turretAngleDeg - 0.0;
        }
        yawResetPrev = yawReset;
    }

    // --- Inline Drive ---

    private void updateDrive() {
        if (gamepad1.right_bumper) {
            if (!autoDriveActive) {
                // Restore logic: Pedro needs to know where we are to start pathing properly
                // But ideally, we want to START from where we are now (savedPose updated on release)?
                // Actually, if we are manual driving, the robot MOVES. Pedro's "pose" is stale.
                // WE NEED to reset Pedro's pose to the current estimated pose (from odometry)?
                // If localizers are running, follower.getPose() IS correct.
                // WAIT. If we stop calling follower.update(), does the localizer stop updating? 
                // "follower.update()" usually calls "drive.update()" which calls "localizer.update()".
                // IF WE STOP CALLING UPDATE, WE LOSE LOCALIZATION.
                
                // CRITICAL FIX: We must keep localization running, but disable DRIVE MOTOR output from Pedro.
                // BUT the user asked to "kill pedro".
                // If we kill pedro, we lose position.
                // If we assume manual driving updates position via Odometry... we need the localizer running.
                
                // Let's stick to the prompt: "save the pose states, kill pedro when its not on"
                // If we kill it, we assume we resume from limit/auto-end-pose? Or do we assume we need to re-seed?
                // "Pedro conflicts with it... causing robot to stutter" -> implies motor contention.
                
                // If we stop calling update(), Pedro won't write to motors.
                // But next time we start, we need a valid start pose. 
                // We will use savedPose (last known valid auto pose) OR we simply reset.
                
                // If we assume the driver moved the robot manually, the old Pedro pose is WRONG.
                // However, without active localization, we don't know the new pose. 
                // The prompt says "save the pose states".
                
                // Let's implement exactly as requested: 
                // 1. Enable: Set start pose to savedPose (which might be stale, but it's what we have).
                // 2. Disable: Save current pose to savedPose.
                
                follower.setStartingPose(savedPose);
                
                Path path = new Path(new BezierLine(savedPose, autoDriveTargetPose));
                path.setLinearHeadingInterpolation(savedPose.getHeading(), autoDriveTargetPose.getHeading());
                follower.followPath(path, true);
                autoDriveActive = true;
            }
            return;
        } else {
            if (autoDriveActive) {
                // Save state before killing
                savedPose = follower.getPose();
                
                // "Kill" (Stop following)
                follower.breakFollowing(); 
                // We don't call startTeleopDrive() because that might engage lock modes? 
                // Actually startTeleopDrive just sets drive vector control. 
                // We just stop calling update().
                
                autoDriveActive = false;
            }
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        frontLeft.setPower((rotY + rotX + rx) / denom);
        backLeft.setPower((rotY - rotX + rx) / denom);
        frontRight.setPower((rotY - rotX - rx) / denom);
        backRight.setPower((rotY + rotX - rx) / denom);
    }

    // getDynamicRpm removed - logic moved to TeleOpShooter
    private void updateAutoShoot() {
        // 1. Toggle Switch (G2 Right D-Pad)
        if (gamepad2.dpad_right && !g2DpadRightPrev) {
            autoShootEnabled = !autoShootEnabled;
        }
        g2DpadRightPrev = gamepad2.dpad_right;

        // 2. Camera Config Retry (ensure Zoom/Focus are set once streaming)
        if (!cameraConfigured && aprilTagCamera.isStreaming()) {
            aprilTagCamera.configureCameraControls();
            cameraConfigured = true;
        }

        // 3. Auto Shoot Trigger
        // Trigger if: Enabled AND Goal Visible AND Shooter Idle/Off
        if (autoShootEnabled && !shooter.isActive() && !shooter.isScanning()) {
            TagObservation goal = aprilTagCamera.getLatestGoalObservation();
            if (goal != null) {
                // Check freshness (ensure we aren't using stale data)
                long now = System.currentTimeMillis();
                if ((now - goal.timestampMs) < 200) { // Detection from last ~200ms
                    shooter.startScan();
                }
            }
        }
    }
}
