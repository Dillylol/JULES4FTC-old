package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

import java.util.List;

/**
 * Bjorn CV Inertial Test
 * 
 * Objective: Test Inertial + CV tracking.
 * Logic:
 * 1. Lock onto AprilTag (Blue or Red Goal).
 * 2. Maintain "Field Heading" to that tag using IMU.
 * 3. Correct "Field Heading" using Camera yaw error (Visual Servoing).
 * 
 * Logic copied from BjornTeleBlue.java
 */
@TeleOp(name = "Bjorn CV Inertial Test", group = "Tests")
public class BjornCVInertialTest extends OpMode {

    // --- Hardware ---
    private DcMotorEx turret;
    private IMU imu;
    private AprilTagCamera aprilTagCamera;

    // --- Constants (Copied from BjornTeleBlue) ---
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 75.52;
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;
    
    private static final double TURRET_KP = 0.015;
    private static final double TURRET_KD = 0.004;
    private static final double ROBOT_ROTATION_FF_GAIN = 0.0055;
    private static final double DEADBAND_DEG = 2.0;
    private static final double MAX_POWER_DELTA = 0.15;
    private static final double LIMIT_MIN = BjornConstants.Motors.TURRET_MIN_DEG;
    private static final double LIMIT_MAX = BjornConstants.Motors.TURRET_MAX_DEG;
    private static final double APRILTAG_CORRECTION_GAIN = 0.5;
    private static final double G2_TURRET_POWER = 1.0;

    // --- State ---
    private double targetFieldHeading = 90.0; // Default to center?
    private double turretAngleDeg = 0.0;
    private int lastEncoderPos = 0;
    private double lastTurretPower = 0.0;
    private double lastTurretError = 0.0;
    private double lastRobotYaw = 0.0;
    private double lastLoopTime = 0;
    private boolean cameraConfigured = false;
    
    // --- Tracking State ---
    private boolean trackingActive = false;
    private int lockedTagId = -1;

    @Override
    public void init() {
        // --- Init Hardware ---
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, BjornConstants.Sensors.IMU);
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);
        imu.resetYaw();

        // --- Camera Init (MJPEG 640x480 by default in AprilTagCamera) ---
        aprilTagCamera = new AprilTagCamera();
        aprilTagCamera.start(hardwareMap, null);

        telemetry.addLine("Initialized. Ensure Camera is MJPEG 640x480.");
        telemetry.addLine("Press Start to begin tracking.");
    }

    @Override
    public void start() {
        lastLoopTime = System.currentTimeMillis() / 1000.0;
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        double now = nowMs / 1000.0;
        double dt = (lastLoopTime > 0) ? (now - lastLoopTime) : 0.02;
        lastLoopTime = now;

        // --- Camera Config Retry Removed for Basic Settings ---
        // if (!cameraConfigured && aprilTagCamera.isStreaming()) {
        //     aprilTagCamera.configureCameraControls();
        //     cameraConfigured = true;
        // }

        // --- Manual Control (D-Pad) ---
        double manualInput = 0.0;
        if (gamepad1.dpad_left) manualInput = -1.0;  // User requested reverse: Left = Decrease
        if (gamepad1.dpad_right) manualInput = 1.0; // User requested reverse: Right = Increase
        
        if (manualInput != 0.0) {
            // Manual Override: Adjust target heading directly
            // Rate: 90 deg/sec
            targetFieldHeading += manualInput * 90.0 * dt;
        }

        // --- Logic Update ---
        updateTracking(dt);
        updateTurret(dt);

        // --- Telemetry ---
        telemetry.addData("Turret Angle", "%.1f°", turretAngleDeg);
        telemetry.addData("Target Heading", "%.1f°", targetFieldHeading);
        telemetry.addData("Robot Yaw", "%.1f°", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Tracking", trackingActive ? ("LOCKED: " + lockedTagId) : "SEARCHING");
        
        // Debug: Show all visible tags
        List<TagObservation> debugDetections = aprilTagCamera.pollDetections();
        StringBuilder tagIds = new StringBuilder();
        for (TagObservation obs : debugDetections) {
            tagIds.append(obs.id).append(" ");
        }
        telemetry.addData("Visible Tags", tagIds.toString());
        telemetry.addData("Cam Stream", aprilTagCamera.isStreaming());
        
        telemetry.update();
    }
    
    private void updateTracking(double dt) {
        List<TagObservation> detections = aprilTagCamera.pollDetections();
        TagObservation target = null;
        
        // Strategy: 
        // 1. If not locked, look for ANY goal tag (Blue or Red). First one seen becomes lock.
        // 2. If locked, look for THAT tag.
        
        if (!trackingActive) {
            for (TagObservation obs : detections) {
                if (CameraConfig.isGoalTag(obs.id)) {
                    lockedTagId = obs.id;
                    trackingActive = true;
                    target = obs;
                    break;
                }
            }
        } else {
            // Already locked, search for specific tag
            boolean found = false;
            for (TagObservation obs : detections) {
                if (obs.id == lockedTagId) {
                    target = obs;
                    found = true;
                    break;
                }
            }
            // If lost for too long? For now, we just don't update if not found.
            // But we keep "trackingActive" true to maintain Inertial Hold on last known direction.
        }

        if (target != null) {
            // --- Visual Servoing Logic (Copied from BjornTeleBlue) ---
            
            // Filter Hallucinations: Check range (max 6.0 meters / ~20 ft)
            // Using Y for Depth based on debug.
            double distMeters = Math.hypot(target.x, target.y); 
            if (distMeters < 6.0) {
                // Compensation for Camera Offset
                // offset (m) / distance (m)
                // We want the Turret Center to point at the Tag Center.
                // desiredBearing is the angle we WANT the tag to be at in the camera frame.
                // NOTE: Data confirmation: Y is Forward/Depth. X is Right(Pos)/Left(Neg).
                // (Definitions moved to bottom for absolute calculation)
                
                // Calculate Actual Bearing from Camera to Tag
                // Using Y as depth (Based on debug data: Y=1.9m vs Z=0.01m)
                // Note: Standard atan2(y, x). Here x=Horizontal, y=Depth.
                // We want 0 when X=0. atan2(0, 1) = 0.
                // If X is Positive (Right), atan2(0.5, 2.0) = Positive.
                double currentBearing = Math.toDegrees(Math.atan2(target.x, target.y));
                
                // Debug Telemetry
                telemetry.addData("Tag Range", "%.2fm", distMeters);
                telemetry.addData("Tag Bearing", "%.1f°", currentBearing);
                telemetry.addData("Raw", "X:%.2f Y:%.2f Z:%.2f", target.x, target.y, target.z);

                // --- INSTANT ABSOLUTE TARGETING ---
                // Instead of slowly integrating the error, we calculate EXACTLY where the tag is 
                // in "Field-Relative Heading" terms and snap to it.
                //
                // Logic:
                // 1. Where is the tag physically relative to the robot chassis?
                //    Angle_Robot_Tag = Turret_Angle_Current + Camera_Bearing_To_Tag
                // 2. What is that in Field Heading?
                //    Field_Heading_Tag = Angle_Robot_Tag - Robot_Yaw_Current
                //
                // This becomes our new Target Field Heading.
                
                double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                
                double absoluteTagAngleRobotFrame = turretAngleDeg + currentBearing;
                double newTargetFieldHeading = absoluteTagAngleRobotFrame - robotYaw;
                
                // Apply Offset for Camera Position?
                // currentBearing points to the tag. desiredBearing was offset.
                // If we want to center the TAG, we use currentBearing.
                // If we want to center the TURRET on the tag (accounting for camera offset),
                // we should adjust.
                // desiredBearing = angle we WANT tag to be at.
                // If tag is at "desiredBearing", then we are aimed correctly.
                // So error = current - desired.
                // If error is 0, we don't move.
                // Using "Instant" math: we want to move such that NewBearing = DesiredBearing.
                // DeltaNeeded = Current - Desired.
                // NewTurret = CurrentTurret + DeltaNeeded.
                
                double offsetMeters = CameraConfig.CAMERA_LATERAL_OFFSET_INCHES * 0.0254;
                double desiredBearing = Math.toDegrees(Math.atan2(offsetMeters, target.y));
                double error = currentBearing - desiredBearing;
                
                // New Goal:
                targetFieldHeading = (turretAngleDeg + error) - robotYaw;

                double minFieldHeading = LIMIT_MIN - robotYaw;
                double maxFieldHeading = LIMIT_MAX - robotYaw;
                targetFieldHeading = Range.clip(targetFieldHeading, minFieldHeading, maxFieldHeading);
            }
        }
    }

    private void updateTurret(double dt) {
        // Update turret angle from encoder
        int currentPos = turret.getCurrentPosition();
        int deltaTicks = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;
        turretAngleDeg += (deltaTicks / TURRET_TICKS_PER_DEGREE) * BjornConstants.Motors.TURRET_ENCODER_DIRECTION;

        // Get robot rotation
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double robotRate = (dt > 0) ? (robotYaw - lastRobotYaw) / dt : 0.0;
        lastRobotYaw = robotYaw;

        // --- Heading Hold PD Control ---
        double desiredTurretAngle = targetFieldHeading + robotYaw;

        // ABSOLUTE SAFETY CLAMP: Ensure target never exceeds physical limits
        desiredTurretAngle = Range.clip(desiredTurretAngle, LIMIT_MIN, LIMIT_MAX);

        double error = desiredTurretAngle - turretAngleDeg;

        if (Math.abs(error) < DEADBAND_DEG) {
            error = 0;
        }

        double derivative = (dt > 0) ? (error - lastTurretError) / dt : 0.0;
        double pid = (error * TURRET_KP) + (derivative * TURRET_KD);
        double ff = robotRate * ROBOT_ROTATION_FF_GAIN;

        double turretPower = pid + ff;
        lastTurretError = error;

        // Apply limits (Standard PID suppression)
        if (turretAngleDeg < LIMIT_MIN && turretPower < 0)
            turretPower = 0;
        if (turretAngleDeg > LIMIT_MAX && turretPower > 0)
            turretPower = 0;

        // Clamp and slew rate limit (Standard)
        if (Double.isNaN(turretPower))
            turretPower = 0;
        turretPower = Range.clip(turretPower, -G2_TURRET_POWER, G2_TURRET_POWER);

        double powerDelta = Range.clip(turretPower - lastTurretPower, -MAX_POWER_DELTA, MAX_POWER_DELTA);
        double slewPower = lastTurretPower + powerDelta;

        // --- FORCE FIELD OVERRIDE (Bypasses Slew Rate) ---
        // If we are out of bounds, we ignore slew rate and apply IMMEDIATE corrective
        // force.
        if (turretAngleDeg < LIMIT_MIN) {
            slewPower = 0.6; // Immediate Hard Push Right
        } else if (turretAngleDeg > LIMIT_MAX) {
            slewPower = -0.6; // Immediate Hard Push Left
        }

        turretPower = slewPower;
        lastTurretPower = turretPower;

        // CORRECTION: User requested flip motor.
        // Changing to -1.0.
        double correctedPowerDirection = -1.0; 
        turret.setPower(turretPower * correctedPowerDirection);
    }
    
    @Override
    public void stop() {
        if (aprilTagCamera != null) {
            aprilTagCamera.close();
        }
    }
}
