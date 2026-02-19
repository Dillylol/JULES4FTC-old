package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

/**
 * Bjorn Hybrid Smart Lock (Fixed)
 * - Uses BjornTeleRed's Visual Servoing (Integrator) for active tracking.
 * - Uses PedroPathing (Odometry) & Pose for target memory.
 * - Replaced missing 'Point' class with 'Pose'.
 */
@TeleOp(name = "Bjorn Smart Lock Test", group = "Test")
public class BjornCVLockTest extends OpMode {

    // Hardware
    private DcMotorEx turret;
    private DcMotorEx lf, rf, lb, rb;
    private AprilTagCamera camera;
    
    // Pedro Follower (Localization)
    private Follower follower; 

    // Constants (From BjornTeleRed)
    private static final int GOAL_TAG_ID = 24; 
    private static final double TURRET_KP = 0.015; 
    private static final double TURRET_KD = 0.004; 
    private static final double ROBOT_ROTATION_FF_GAIN = 0.0055;
    private static final double DEADBAND_DEG = 0.5;
    private static final double LIMIT_MIN = BjornConstants.Motors.TURRET_MIN_DEG;
    private static final double LIMIT_MAX = BjornConstants.Motors.TURRET_MAX_DEG;
    private static final double MANUAL_RATE_DEG_PER_SEC = 90.0;
    private static final double APRILTAG_CORRECTION_GAIN = 0.5;

    // State
    private int lastEncoderPos = 0;
    private double turretAngleDeg = 0.0;
    private double targetFieldHeading = 90.0; // Heading of Turret relative to Field
    private Pose targetPose = null; // FIELD Coordinate of Target (using Pose as Point)
    
    // Control State
    private double lastError = 0.0;
    private double lastLoopTime = 0.0;
    private double lastRobotYaw = 0.0;
    private double manualOffset = 0.0;

    @Override
    public void init() {
        // --- Drive Motors ---
        lf = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.FRONT_LEFT);
        rf = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.FRONT_RIGHT);
        lb = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.BACK_LEFT);
        rb = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.BACK_RIGHT);
        
        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);

        // --- Turret ---
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Pedro Follower ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0)); // Start at 0,0,0

        // --- Camera ---
        camera = new AprilTagCamera();
        camera.start(hardwareMap, null);
        
        telemetry.addLine("Bjorn Hybrid Smart Lock Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        try {
            if (camera.isStreaming()) camera.setManualExposure(5, 250);
        } catch (Exception ignored) {}
        
        follower.startTeleopDrive();
        lastLoopTime = System.currentTimeMillis();
        lastEncoderPos = turret.getCurrentPosition();
        turretAngleDeg = 0.0;
        targetFieldHeading = 90.0; // Default Forward
        manualOffset = 0.0;
    }

    @Override
    public void loop() {
        // Time Delta
        double nowMs = System.currentTimeMillis();
        double dt = (nowMs - lastLoopTime) / 1000.0;
        if (dt == 0) dt = 0.001;
        lastLoopTime = nowMs;

        // 1. Update Subsystems
        follower.update();
        Pose robotPose = follower.getPose();
        double robotYaw = Math.toDegrees(robotPose.getHeading()); // Pedro uses Radians
        
        // Turret State
        int currentPos = turret.getCurrentPosition();
        int deltaTicks = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;
        double ticksPerDegree = org.firstinspires.ftc.teamcode.common.BjornHardware.TURRET_TICKS_PER_DEGREE;
        turretAngleDeg += (deltaTicks / ticksPerDegree) * BjornConstants.Motors.TURRET_ENCODER_DIRECTION;

        // 2. Camera Processing
        TagObservation tag = null;
        List<TagObservation> dets = camera.pollDetections();
        for (TagObservation obs : dets) {
            if (obs.id == GOAL_TAG_ID) {
                tag = obs;
                break;
            }
        }
        
        // 3. Update Targeting Logic
        // Manual override (D-Pad) updates an OFFSET
        if (gamepad1.dpad_left) manualOffset += MANUAL_RATE_DEG_PER_SEC * dt;
        if (gamepad1.dpad_right) manualOffset -= MANUAL_RATE_DEG_PER_SEC * dt;

        if (tag != null) {
            // --- VISIBLE: Use BjornTeleRed Visual Servoing ---
            
            double distMeters = Math.hypot(tag.x, tag.y);
            
            // Compensation (From BjornTeleRed)
            double offsetMeters = CameraConfig.CAMERA_LATERAL_OFFSET_INCHES * 0.0254;
            double desiredYaw = Math.toDegrees(Math.atan2(-offsetMeters, tag.y));
            double yawError = tag.yaw - desiredYaw;
            
            // Integrate Logic (Directly from TeleRed)
            double correction = yawError * APRILTAG_CORRECTION_GAIN * dt * 30.0 * CameraConfig.CAMERA_TO_TURRET_SCALAR;
            targetFieldHeading += correction; 
            
            // Sync Memory (Target Pose)
            double currentLookAngle = robotPose.getHeading() + Math.toRadians(turretAngleDeg); 
             // Note: Rough estimation. Usually TurretDeg is relative to robot. 
             // And RobotHeading is relative to field.
             // If Turret 0 is "Robot Forward" (or whatever reference), adding them gives Field Angle.
            
            double distInch = distMeters * 39.37;
            double tx = robotPose.getX() + distInch * Math.cos(currentLookAngle);
            double ty = robotPose.getY() + distInch * Math.sin(currentLookAngle);
            targetPose = new Pose(tx, ty, 0);
            
        } else if (targetPose != null) {
            // --- INVISIBLE: Use Odometry Point Tracking ---
            // Calculate Angle to stored Target Point
            double dx = targetPose.getX() - robotPose.getX();
            double dy = targetPose.getY() - robotPose.getY();
            
            // Absolute Field Angle to Target
            double absAngleRad = Math.atan2(dy, dx);
            double absAngleDeg = Math.toDegrees(absAngleRad);
            
            // Update the integrator state:
            // "targetFieldHeading" in BjornTeleRed seems to track "Turret Angle in Field Frame" (roughly).
            // We set it such that: DesiredTurretAngle (Local) = TargetFieldHeading + RobotYaw (Bjorn Formula? No).
            // Bjorn Formula: desired = targetField + robotYaw.
            // THIS IMPLIES `targetFieldHeading` IS NOT FIELD RELATIVE in the standard sense?
            // If RobotYaw is 0, Desired = targetField.
            // If I want to point North (90), and Robot is 0, Desired=90. So targetField=90.
            // If Robot is North (90), to point North (90), Desired should be 0 (Forward on Robot)?
            // Formula: 0 = 90 + 90? No, 180.
            // This implies Bjorn's `robotYaw` might be inverted or `targetFieldHeading` logic differs.
            
            // Let's assume WE WANT correct geometry:
            // Desired Local = Absolute Field Angle - Robot Field Angle.
            // Bjorn's line: `desired = targetField + robotYaw`.
            // Use OUR calculation for Desired Local:
            double desiredLocalDeg = absAngleDeg - robotYaw;
            
            // Now back-solve `targetFieldHeading` to satisfy Bjorn's formula if we use it for consistency?
            // `desiredLocalDeg = targetField + robotYaw` -> `targetField = desiredLocalDeg - robotYaw`.
            // `targetField = (Abs - Robot) - Robot = Abs - 2*Robot`.
            targetFieldHeading = desiredLocalDeg - robotYaw;
        }

        // Apply Manual Offset
        targetFieldHeading += manualOffset;

        // 4. Calculate PID Target (BjornTeleRed Logic)
        double desiredTurretAngle = targetFieldHeading + robotYaw;

        // 5. PID & Clamping
        desiredTurretAngle = Range.clip(desiredTurretAngle, LIMIT_MIN, LIMIT_MAX);

        double error = desiredTurretAngle - turretAngleDeg;
        
        double derivative = (error - lastError) / dt;
        double pid = (error * TURRET_KP) + (derivative * TURRET_KD);
        double turretPower = pid; 
        lastError = error;
        
        turretPower = Range.clip(turretPower, -1.0, 1.0);
        turret.setPower(turretPower * BjornConstants.Motors.TURRET_POWER_DIRECTION);

        // 6. Manual Drive
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double heading = robotPose.getHeading();
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        lf.setPower((rotY + rotX + rx) / denom);
        lb.setPower((rotY - rotX + rx) / denom);
        rf.setPower((rotY - rotX - rx) / denom);
        rb.setPower((rotY + rotX - rx) / denom);

        // Telemetry
        telemetry.addData("Mode", tag != null ? "VISIBLE" : "MEMORY");
        telemetry.addData("Tag", tag != null ? "LOCKED" : "LOST");
        telemetry.addData("Target Pose", targetPose != null ? String.format("%.0f, %.0f", targetPose.getX(), targetPose.getY()) : "None");
        telemetry.addData("Turret Ang", "%.1f", turretAngleDeg);
        telemetry.update();
    }
}
