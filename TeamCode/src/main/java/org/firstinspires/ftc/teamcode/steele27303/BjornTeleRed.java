package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.configurables.TurretConfigurables;
import org.firstinspires.ftc.teamcode.common.turret.TurretControl;

import java.util.List;

/**
 * RED TeleOp - Drive/IMU/Pedro/Camera/Turret Inline (Reduced Latency)
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
@TeleOp(name = "Bjorn TeleOp RED")
public class BjornTeleRed extends BjornTeleBase {

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
    private static final int GOAL_TAG_ID = CameraConfig.RED_GOAL_TAG_ID;

    // --- Turret Constants (from TurretConfigurables where possible) ---
    private static final double LIMIT_MIN = BjornConstants.Motors.TURRET_MIN_DEG;
    private static final double LIMIT_MAX = BjornConstants.Motors.TURRET_MAX_DEG;
    private static final double MANUAL_RATE_DEG_PER_SEC = 90.0;
    private static final double G2_TURRET_POWER = 1.0;
    private static final double G1_TURRET_POWER = 0.5;

    // --- Turret State ---
    private double targetFieldHeading = 90.0;
    private double turretAngleDeg = 0.0;
    private boolean positionTrackingEnabled = false;
    private boolean wasManualControl = false;
    private boolean autoShootEnabled = false;
    private boolean cameraConfigured = false;

    // --- Turret Control System ---
    private TurretControl turretControl;

    // --- Input State ---
    private boolean g1LbPrev = false;
    private boolean dpadDownPrev = false;
    private boolean isWaitingForRecalibration = false;
    private boolean g2APrev = false;
    private boolean g2DpadUpPrev = false;
    private boolean dpadUpPrev = false; // Added for G1 Camera Toggle
    private double lastLoopTime = 0;

    // --- Alliance Config ---
    private static final Pose AUTO_END_POSE = BjornConstants.Auto.RED_AUTO_END_POSE;

    @Override
    protected String getAllianceName() {
        return "RED";
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
        follower = Constants.createFollower(hardwareMap);
        // Start at Auto end pose for seamless transition (Hardcoded handoff)
        Pose startPose = BjornConstants.Auto.RED_AUTO_END_POSE;
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

        // GoalCalculator removed — turret now uses camera tracking

        // Pass Camera to Shooter for CV calculation
        shooter.setCamera(aprilTagCamera, GOAL_TAG_ID);

        telemetry.addLine("Bjorn TeleOp RED Initialized");
        telemetry.addData("Goal Tag", GOAL_TAG_ID);
        telemetry.update();
    }

    @Override
    public void start() {
        if (follower != null) follower.startTeleopDrive();
    }

    private Pose getCurrentPose() {
        Pose p = follower.getPose();
        return p != null ? p : savedPose;
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        double now = nowMs / 1000.0;
        double dt = (lastLoopTime > 0) ? (now - lastLoopTime) : 0.02;
        lastLoopTime = now;

        // --- Update Systems (Inline) ---
        // Always update Pedro follower to maintain localization (needed for position
        // tracking)
        follower.update();

        // Feed current robot pose to turret control for position tracking
        turretControl.updateRobotPose(getCurrentPose());

        // --- Input Handling ---
        handleRecalibration();

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
        telemetry.addData("Alliance", "RED");

        telemetry.addLine("--- TURRET ---");
        telemetry.addData("Mode", positionTrackingEnabled ? "CAMERA TRACK" : "MANUAL");
        telemetry.addData("Turret Angle", "%.1f°", turretControl.getCurrentAngle());
        telemetry.addData("Turret Target", "%.1f°", turretControl.getTargetAngle());
        double trackError = turretControl.getTargetAngle() - turretControl.getCurrentAngle();
        telemetry.addData("Tracking Error", "%.1f°", trackError);

        telemetry.addLine("--- CAMERA ---");
        TagObservation latestGoal = aprilTagCamera.getLatestGoalObservation();
        telemetry.addData("Camera", aprilTagCamera.isStreaming() ? "STREAMING" : "OFF");
        telemetry.addData("Exposure Locked", cameraConfigured ? "YES" : "NO");
        if (latestGoal != null) {
            double tagDist = Math.hypot(latestGoal.x, latestGoal.y);
            double tagBearing = Math.toDegrees(Math.atan2(latestGoal.x, latestGoal.y));
            telemetry.addData("Tag ID", latestGoal.id);
            telemetry.addData("Tag Range", "%.2f m", tagDist);
            telemetry.addData("Tag Bearing", "%.1f°", tagBearing);
        } else {
            telemetry.addData("Tag", "NOT VISIBLE");
        }
        telemetry.addData("Detections", aprilTagCamera.getLastDetectionCount());

        telemetry.addLine("--- SYSTEMS ---");
        telemetry.addData("Vision Calibrate", isWaitingForRecalibration ? "WAITING FOR TAG..." : "IDLE");
        telemetry.addData("Auto-Shoot", autoShootEnabled ? "ON" : "OFF");
        addSubsystemTelemetry(nowMs);
        telemetry.update();
    }

    // --- Turret Control (Inline - From InclusiveTurretTest) ---

    private void updateTurret(double dt) {
        long nowMs = System.currentTimeMillis();

        turretAngleDeg = turretControl.getCurrentAngle();

        // --- G2: A toggles Position Tracking (now Camera Tracking) ---
        if (gamepad2.a && !g2APrev) {
            positionTrackingEnabled = !positionTrackingEnabled;
            turretControl.setPositionTrackingEnabled(positionTrackingEnabled);
        }
        g2APrev = gamepad2.a;
        dpadUpPrev = gamepad1.dpad_up;

        // --- SAFETY LOCKOUT: Manual control is ONLY allowed when tracking is OFF ---
        if (!positionTrackingEnabled) {
            // --- Manual Control ---
            // G2: Left stick X (full authority)
            // G1: D-Pad L/R (limited authority)
            double g2Input = gamepad2.left_stick_x;
            double g1Input = 0.0;
            if (gamepad1.dpad_left)
                g1Input = -1.0;
            else if (gamepad1.dpad_right)
                g1Input = 1.0;

            // Combine inputs (G2 has priority if both active)
            double stickInput = (Math.abs(g2Input) > 0.1) ? g2Input : g1Input * (G1_TURRET_POWER / G2_TURRET_POWER);
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
        } else {
            // --- Camera Tracking: Use AprilTag bearing to aim turret ---
            // Poll camera and look for goal tag
            List<TagObservation> detections = aprilTagCamera.pollDetections();
            TagObservation goalTag = null;
            for (TagObservation obs : detections) {
                if (obs.id == GOAL_TAG_ID) {
                    goalTag = obs;
                    break;
                }
            }

            if (goalTag != null) {
                // Tag visible — calculate bearing error
                double distMeters = Math.hypot(goalTag.x, goalTag.y);

                if (distMeters < 6.0 && distMeters > 0.1) {
                    // Current bearing from camera to tag (atan2(horizontal, depth))
                    double currentBearing = Math.toDegrees(Math.atan2(goalTag.x, goalTag.y));

                    // Desired bearing accounts for camera lateral offset
                    double offsetMeters = CameraConfig.CAMERA_LATERAL_OFFSET_INCHES * 0.0254;
                    double desiredBearing = Math.toDegrees(Math.atan2(offsetMeters, goalTag.y));

                    // Error = how far off the tag is from where we want it
                    double bearingError = (currentBearing - desiredBearing)
                            * TurretConfigurables.cameraCorrectionGain
                            * CameraConfig.CAMERA_TO_TURRET_SCALAR;

                    // Adjust turret target directly: current angle + error
                    double newTarget = turretAngleDeg + bearingError;
                    newTarget = Range.clip(newTarget, LIMIT_MIN, LIMIT_MAX);
                    turretControl.setTargetAngle(newTarget);
                    targetFieldHeading = newTarget; // Sync for telemetry
                }
                // If distance out of range, hold last target (do nothing)
            }
            // If tag not visible, hold last target angle (no triangulation fallback)

            wasManualControl = false; // Reset so manual doesn't snap on toggle-off
        }

        // Keep camera polling active even when not tracking (for shooter/auto-shoot)
        if (!positionTrackingEnabled) {
            aprilTagCamera.pollDetections();
        }

        // --- Run TurretControl ---
        // TurretControl.update() handles PD, FF, force field, limits,
        // and applies TURRET_POWER_DIRECTION internally via setPower().
        // Do NOT set motor power again here — it would double-invert.
        turretControl.update(dt);
    }

    // --- Inline Input Handlers ---

    private void handleRecalibration() {
        boolean dpadDown = gamepad2.dpad_down;
        if (dpadDown && !dpadDownPrev) {
            isWaitingForRecalibration = !isWaitingForRecalibration;
        }
        dpadDownPrev = dpadDown;

        if (isWaitingForRecalibration) {
            TagObservation goal = aprilTagCamera.getLatestGoalObservation();
            long now = System.currentTimeMillis();
            boolean seesTag = (goal != null && (now - goal.timestampMs) < 250);

            if (seesTag) {
                // Determine Field Position from AprilTag
                double robotHeading = getCurrentPose().getHeading(); // Radians from Odometry
                double turretAngleField = robotHeading + Math.toRadians(turretAngleDeg);
                
                // Tag Coords Relative to Turret Center (X = Forward, Y = Left) in inches
                double x_turret = goal.y * 39.37;
                double y_turret = -goal.x * 39.37 + CameraConfig.CAMERA_LATERAL_OFFSET_INCHES;
                
                // Vector from Turret Center to Tag (Field Coords)
                double dx_field = x_turret * Math.cos(turretAngleField) - y_turret * Math.sin(turretAngleField);
                double dy_field = x_turret * Math.sin(turretAngleField) + y_turret * Math.cos(turretAngleField);
                
                // Turret Center Absolute Field Coords
                double tx = BjornConstants.FieldPositions.RED_CALIBRATION_TAG_X - dx_field;
                double ty = BjornConstants.FieldPositions.RED_CALIBRATION_TAG_Y - dy_field;
                
                // Robot Center Absolute Field Coords
                double dx_rob = BjornConstants.Motors.TURRET_OFFSET_X * Math.cos(robotHeading) - BjornConstants.Motors.TURRET_OFFSET_Y * Math.sin(robotHeading);
                double dy_rob = BjornConstants.Motors.TURRET_OFFSET_X * Math.sin(robotHeading) + BjornConstants.Motors.TURRET_OFFSET_Y * Math.cos(robotHeading);
                
                double rx = tx - dx_rob;
                double ry = ty - dy_rob;
                
                // Maintain heading but snap position to camera reading
                follower.setStartingPose(new Pose(rx, ry, robotHeading));
                
                // Done calibrating!
                isWaitingForRecalibration = false;
            }
        }
    }

    // --- Inline Drive ---

    private void updateDrive() {
        /*
        if (gamepad1.right_bumper) {
            if (!autoDriveActive) {
                // Restore logic: Use savedPose to restart Pedro pathing
                // This assumes manual driving does not update Pedro's pose (since update() is
                // paused).
                // We use the last known "good" pose from when auto was active or initialized.

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
                savedPose = getCurrentPose();

                // "Kill" (Stop following)
                follower.breakFollowing();
                follower.startTeleopDrive();

                autoDriveActive = false;
            }
        }
        */

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        // Pedro handles field-centric math natively when boolean flag is false
        follower.setTeleOpDrive(y, x, rx, false);
    }

    // getDynamicRpm removed - logic moved to TeleOpShooter
    private void updateAutoShoot() {
        // 1. Toggle Switch (G2 Right D-Pad)
        if (gamepad2.dpad_up && !g2DpadUpPrev) {
            autoShootEnabled = !autoShootEnabled;
        }
        g2DpadUpPrev = gamepad2.dpad_up;

        // 2. Camera Config Retry (lock exposure/gain once streaming)
        if (!cameraConfigured && aprilTagCamera.isStreaming()) {
            aprilTagCamera.configureCameraControls();
            aprilTagCamera.setManualExposure(CameraConfig.TUNED_EXPOSURE, CameraConfig.TUNED_GAIN);
            cameraConfigured = true;
        }

        // 3. Auto Shoot Trigger
        if (autoShootEnabled && !shooter.isActive()) {
            TagObservation goal = aprilTagCamera.getLatestGoalObservation();
            boolean seesTag = false;
            if (goal != null) {
                // Check freshness (ensure we aren't using stale data)
                long now = System.currentTimeMillis();
                if ((now - goal.timestampMs) < 200) { // Detection from last ~200ms
                    seesTag = true;
                }
            }
            shooter.setIdle(seesTag);
        }
    }
}
