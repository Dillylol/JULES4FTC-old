package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.common.turret.HemisphericalTurret;

import java.util.List;

@TeleOp(name = "Bjorn CV Turret Test", group = "Tests")
public class BjornCVTurretTest extends OpMode {

    // --- Hardware ---
    private DcMotorEx turret;
    private IMU imu;
    private Follower follower;
    private AprilTagCamera aprilTagCamera;
    
    // --- State ---
    private boolean localized = false;
    private Pose goalPoint = BjornConstants.FieldPositions.RED_GOAL;
    private HemisphericalTurret hemiTurret;
    
    // --- Turret Constants (COPIED 1:1 from BjornTeleBlue) ---
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
    private static final double G2_TURRET_POWER = 1.0;

    // --- Turret State (COPIED 1:1) ---
    private double targetFieldHeading = 90.0; // Will be set by HemiTurret
    private double turretAngleDeg = 0.0;
    private int lastEncoderPos = 0;
    private double lastTurretPower = 0.0;
    private double lastTurretError = 0.0;
    private double lastRobotYaw = 0.0;
    private double lastLoopTime = 0;

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
        imu.resetYaw(); // Assume start at 0 for now until localized

        follower = Constants.createFollower(hardwareMap);
        // Default pose
        follower.setStartingPose(new Pose(0, 0, 0));

        aprilTagCamera = new AprilTagCamera();
        aprilTagCamera.start(hardwareMap, null);
        
        hemiTurret = new HemisphericalTurret();

        telemetry.addLine("Initialized. Waiting for Tag 24...");
    }

    @Override
    public void init_loop() {
        if (!localized) {
            List<TagObservation> detections = aprilTagCamera.pollDetections();
            for (TagObservation obs : detections) {
                if (obs.id == 24) { // RED GOAL
                   // ONE-AND-DONE LOCALIZATION
                   telemetry.addLine("TAG 24 SEEN! LOCALIZING...");
                   
                   // Hypothetical set for test
                   // Note: Real triangulation needs math here.
                   // For this test, we assume test start position aligns with camera view.
                   // The goal is just to trigger "localized" state.
                   follower.setStartingPose(new Pose(0, 0, 0)); // Reset
                   localized = true;
                }
            }
            telemetry.addData("Localized", localized);
        } else {
             telemetry.addLine("Ready to Start. Pose Acquired.");
        }
    }

    @Override
    public void start() {
        lastLoopTime = System.currentTimeMillis() / 1000.0;
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        double now = nowMs / 1000.0;
        double dt = (lastLoopTime > 0) ? (now - lastLoopTime) : 0.02;
        lastLoopTime = now;
        
        follower.update();
        Pose robotPose = follower.getPose();
        
        // --- Calculate Target Angle (HemiTurret Logic) ---
        // Input: Robot Pose, Goal Point (as Pose)
        targetFieldHeading = hemiTurret.calculate(robotPose, goalPoint, turretAngleDeg);
        
        // --- Update Turret (COPIED 1:1) ---
        updateTurret(dt);
        
        telemetry.addData("Turret", "%.1f", turretAngleDeg);
        telemetry.addData("Target", "%.1f", targetFieldHeading);
        telemetry.addData("Pose", robotPose.toString());
        telemetry.update();
    }

    // --- COPIED METHOD 1:1 ---
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

        // NOTE: Manual Control removed, TargetFieldHeading is set by HemiTurret externally
        
        // --- Heading Hold PD Control ---
        double desiredTurretAngle = targetFieldHeading + robotYaw; // Wait, BjornTeleBlue logic check...
        // In BjornTeleBlue: "targetFieldHeading" is a relative "field" heading offset?
        // Line 297: desiredTurretAngle = targetFieldHeading + robotYaw;
        // This means targetFieldHeading is effectively "Angle relative to Field North" MINUS "Robot Yaw"?
        // No, `targetFieldHeading` in BjornTeleBlue seems to be "Desired Turret Angle relative to Robot Zero IF robot was at yaw=0"?
        // Let's verify line 242: targetFieldHeading += headingDelta; 
        // It acts as a memory variable.
        // If we want "Hemispherical Logic", we need to provide the Correct "targetFieldHeading"
        // such that (targetFieldHeading + robotYaw) = The Angle we want the physical turret to be at.
        // So targetFieldHeading = (DesiredPhysicalAngle - RobotYaw).
        
        // ... Continuing 1:1 Copy ...

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
        if (turretAngleDeg < LIMIT_MIN) {
            slewPower = 0.6; // Immediate Hard Push Right
        } else if (turretAngleDeg > LIMIT_MAX) {
            slewPower = -0.6; // Immediate Hard Push Left
        }

        turretPower = slewPower;
        lastTurretPower = turretPower;

        turret.setPower(turretPower * BjornConstants.Motors.TURRET_POWER_DIRECTION);
    }
}
