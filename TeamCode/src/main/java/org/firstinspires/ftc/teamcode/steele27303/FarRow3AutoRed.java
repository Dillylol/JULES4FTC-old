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

@Autonomous(name = "farRow3AutoRed")
public class FarRow3AutoRed extends OpMode {

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
        // Start pose based on the first path start
        follower.setStartingPose(new Pose(87.000, 6.000, Math.toRadians(0)));
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

        goalCalculator = new GoalCalculator(144.0, 144.0);
        
        // --- Initialize Hardware & Shooter ---
        hardwareWrapper = BjornHardware.forTeleOp(hardwareMap);
        shooter = new AutoShooter(hardwareWrapper);
        shooter.setTargetRpm(1000); // Trigger RPM idle mode on start
        
        // Fast 2-second ramp for Auto
        ShooterConfigurables.rampDurationMs = 2000;

        telemetry.addLine("Far Row 3 Auto Red Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(paths.startpreloadshoot, true);
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        long nowMs = System.currentTimeMillis();
        double now = nowMs / 1000.0;
        dt = (lastLoopTime > 0) ? (now - lastLoopTime) : 0.02;
        lastLoopTime = now;

        turretControl.updateRobotPose(follower.getPose());

        autonomousPathUpdate();

        turretControl.update(dt);
        shooter.update();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret Target", turretControl.getTargetAngle());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        if (pathState < 4) {
            turretControl.setTargetMode(TurretControl.TargetMode.WORLD);
            turretControl.setTargetAngle(37.0);
        } else {
            turretControl.setTargetMode(TurretControl.TargetMode.LOCAL);
            turretControl.setTargetAngle(90.0);
        }

        long nowMs = System.currentTimeMillis();

        switch (pathState) {
            case 0:
                if (Math.hypot(follower.getPose().getX() - 87.0, follower.getPose().getY() - 99.0) < 25.0) {
                    shooter.setTargetDistance(47.0);
                    shooter.setIntakePower(true);
                }
                if (!follower.isBusy()) {
                    shooter.setTargetDistance(47.0);
                    shooter.setIntakePower(true);
                    shooter.setShooting(true);
                    actionTimer = nowMs;
                    pathState = 1;
                }
                break;
            case 1:
                if (nowMs - actionTimer > 5000) {
                    shooter.setTargetRpm(1000);
                    shooter.setIntakePower(false);
                    shooter.setShooting(false);
                    follower.followPath(paths.row2shootrow3approach, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.row3approw3capture, true);
                    shooter.setIntakePower(true);
                    shooter.setTargetRpm(0); // Shooter no longer needed
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    // Do not turn off intake immediately, extend intake while reversing
                    follower.followPath(paths.row3capturepark, true);
                    pathState = 4;
                }
                break;
            case 4:
                // Extended intake timing for the final capture before parking
                if (Math.hypot(follower.getPose().getX() - 114.0, follower.getPose().getY() - 39.0) < 10.0) {
                    shooter.setIntakePower(true);
                } else {
                    shooter.setIntakePower(false);
                }
                
                if (!follower.isBusy()) {
                    telemetry.addLine("Path Chain Complete");
                    pathState = 5; // End state
                }
                break;
            case 5:
                break;
        }
    }

    public static class Paths {
        public PathChain startpreloadshoot;
        public PathChain row2shootrow3approach;
        public PathChain row3approw3capture;
        public PathChain row3capturepark;

        public Paths(Follower follower) {
            startpreloadshoot = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(87.000, 6.000),
                            new Pose(87.000, 99.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(350))
                    .build();

            row2shootrow3approach = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(87.000, 99.000),
                            new Pose(84.000, 42.000),
                            new Pose(89.000, 39.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(0))
                    .build();

            row3approw3capture = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(89.000, 39.000),
                            new Pose(114.000, 39.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            row3capturepark = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(114.000, 39.000),
                            new Pose(99.000, 42.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}
