package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.shooter.AutoShooter;
import org.firstinspires.ftc.teamcode.configurables.ShooterConfigurables;
import org.firstinspires.ftc.teamcode.common.turret.TurretControl;
import org.firstinspires.ftc.teamcode.common.turret.GoalCalculator;

@Autonomous(name = "2rowAutoBlue")
public class TwoRowAutoBlue extends OpMode {

    private Follower follower;
    private Paths paths;
    private int pathState = 0;

    private TurretControl turretControl;
    private GoalCalculator goalCalculator;
    private IMU imu;
    private DcMotorEx turretMotor;

    private BjornHardware hardwareWrapper;
    private AutoShooter shooter;
    private long actionTimer = 0;

    private double lastLoopTime = 0.0;
    private double dt = 0.02;

    @Override
    public void init() {
        // --- Initialize Pedro Follower ---
        follower = Constants.createFollower(hardwareMap);
        // Start pose based on the first path start: (33.500, 135.500) heading 180
        follower.setStartingPose(new Pose(33.500, 135.500, Math.toRadians(180)));
        paths = new Paths(follower);

        // --- Initialize IMU ---
        imu = hardwareMap.get(IMU.class, BjornConstants.Sensors.IMU);
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        // --- Initialize Turret ---
        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret");
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Initialize Turret Control System ---
        turretControl = new TurretControl(turretMotor, imu);

        // Target tracking score zone: x = 0, y = 144 (User requested blue score zone)
        goalCalculator = new GoalCalculator(0.0, 144.0);
        
        // --- Initialize Hardware & Shooter ---
        hardwareWrapper = BjornHardware.forTeleOp(hardwareMap);
        shooter = new AutoShooter(hardwareWrapper);
        shooter.setTargetRpm(1000); // Trigger RPM idle mode on start
        
        // Fast 2-second ramp for Auto
        ShooterConfigurables.rampDurationMs = 2000;

        telemetry.addLine("Bjorn Auto Blue Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start the first path immediately
        follower.followPath(paths.startpreloadshoot, true);
        pathState = 0;
    }

    @Override
    public void loop() {
        // --- Updates ---
        follower.update();
        long nowMs = System.currentTimeMillis();
        double now = nowMs / 1000.0;
        dt = (lastLoopTime > 0) ? (now - lastLoopTime) : 0.02;
        lastLoopTime = now;

        // Feed Robot Pose to Turret Control
        turretControl.updateRobotPose(follower.getPose());

        // --- Autonomous Path Logic & Turret Targeting ---
        autonomousPathUpdate();

        // Update Turret Control Loop
        turretControl.update(dt);
        
        // Update Shooter
        shooter.update();

        // --- Telemetry ---
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret Target", turretControl.getTargetAngle());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        // By default during the program, hold a static 143-degree field-relative angle rather than tracking the goal dynamically
        if (pathState < 11) {
            turretControl.setTargetMode(TurretControl.TargetMode.WORLD);
            turretControl.setTargetAngle(143.0);
        } else {
            // When parking (after row2shootrow3approach), set turret heading to 90 degrees local.
            turretControl.setTargetMode(TurretControl.TargetMode.LOCAL);
            turretControl.setTargetAngle(90.0);
        }

        long nowMs = System.currentTimeMillis();

        switch (pathState) {
            case 0:
                // Pre-fire spin up 1 second early (approx 25 inches away from target)
                if (Math.hypot(follower.getPose().getX() - 57.0, follower.getPose().getY() - 99.0) < 25.0) {
                    shooter.setTargetDistance(47.0);
                    shooter.setIntakePower(true);
                }
                if (!follower.isBusy()) {
                    // Arrived at startpreloadshoot. Start shooting!
                    shooter.setTargetDistance(47.0);
                    shooter.setIntakePower(true);
                    shooter.setShooting(true);
                    actionTimer = nowMs;
                    pathState = 1;
                }
                break;
            case 1:
                if (nowMs - actionTimer > 5000) {
                    // 4 second window out. Return to idle, stop intake, and go to next point
                    shooter.setTargetRpm(1000);
                    shooter.setIntakePower(false);
                    shooter.setShooting(false);
                    follower.followPath(paths.preloadshootrow1approach, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.row1approw1capture, true);
                    shooter.setIntakePower(true); // Turn on intake for capture
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    // Do not turn off intake immediately, extend intake while reversing
                    follower.followPath(paths.row1captureshootrow1, true);
                    pathState = 4;
                }
                break;
            case 4:
                // Extended intake timing (keep intaking for first 10 inches of reverse to ensure full capture)
                if (Math.hypot(follower.getPose().getX() - 14.0, follower.getPose().getY() - 86.0) < 10.0) {
                    shooter.setIntakePower(true);
                }
                // Pre-fire spin up 1 second early
                else if (Math.hypot(follower.getPose().getX() - 57.0, follower.getPose().getY() - 99.0) < 25.0) {
                    shooter.setTargetDistance(47.0);
                    shooter.setIntakePower(true);
                }
                else {
                    shooter.setIntakePower(false);
                }
                
                if (!follower.isBusy()) {
                    // Arrived at row1captureshootrow1. Shoot loop again!
                    shooter.setTargetDistance(47.0);
                    shooter.setIntakePower(true);
                    shooter.setShooting(true);
                    actionTimer = nowMs;
                    pathState = 5;
                }
                break;
            case 5:
                if (nowMs - actionTimer > 5000) {
                    shooter.setTargetRpm(1000);
                    shooter.setIntakePower(false);
                    shooter.setShooting(false);
                    follower.followPath(paths.shootrow1row2approach, true);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.row2approw2capture, true);
                    shooter.setIntakePower(true); // Turn on intake for capture
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.row3capturepark, true);
                    pathState = 8;
                }
                break;
            case 8:
                // Extended intake timing for the final capture before parking
                if (Math.hypot(follower.getPose().getX() - 12.0, follower.getPose().getY() - 62.0) < 10.0) {
                    shooter.setIntakePower(true);
                } else {
                    shooter.setIntakePower(false);
                }
                
                if (!follower.isBusy()) {
                    telemetry.addLine("Path Chain Complete");
                    pathState = 13; // End state
                }
                break;
            case 9:
            case 10:
            case 11:
            case 12:
            case 13:
                // End state - do nothing
                break;
        }
    }

    public static class Paths {
        public PathChain startpreloadshoot;
        public PathChain preloadshootrow1approach;
        public PathChain row1approw1capture;
        public PathChain row1captureshootrow1;
        public PathChain shootrow1row2approach;
        public PathChain row2approw2capture;
        public PathChain row2capturerow2shoot;
        public PathChain row2shootrow3approach;
        public PathChain row3approw3capture;
        public PathChain row3capturepark;

        public Paths(Follower follower) {
            startpreloadshoot = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(33.500, 135.500),
                            new Pose(57.000, 99.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-170))
                    .build();

            preloadshootrow1approach = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(57.000, 99.000),
                            new Pose(57.000, 87.000),
                            new Pose(54.000, 86.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-170), Math.toRadians(180))
                    .build();

            row1approw1capture = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(54.000, 86.000),
                            new Pose(14.000, 86.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            row1captureshootrow1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(14.000, 86.000),
                            new Pose(57.000, 99.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-170))
                    .build();

            shootrow1row2approach = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(57.000, 99.000),
                            new Pose(57.000, 68.000),
                            new Pose(56.000, 62.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-170), Math.toRadians(180))
                    .build();

            row2approw2capture = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56.000, 62.000),
                            new Pose(12.000, 62.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            row3capturepark = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(12.000, 62.000),
                            new Pose(45.000, 42.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}
