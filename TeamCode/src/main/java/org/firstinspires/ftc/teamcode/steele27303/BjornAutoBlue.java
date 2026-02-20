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
import org.firstinspires.ftc.teamcode.common.turret.TurretControl;
import org.firstinspires.ftc.teamcode.common.turret.GoalCalculator;

@Autonomous(name = "Bjorn Auto Blue", group = "Blue")
public class BjornAutoBlue extends OpMode {

    private Follower follower;
    private Paths paths;
    private int pathState = 0;

    private TurretControl turretControl;
    private GoalCalculator goalCalculator;
    private IMU imu;
    private DcMotorEx turretMotor;

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

        // Target tracking score zone: x = 12, y = 138 (User requested blue score zone)
        goalCalculator = new GoalCalculator(12.0, 138.0);

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

        // --- Telemetry ---
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret Target", turretControl.getTargetAngle());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        // By default during the program, track the goal
        if (pathState < 9) {
            double goalAngle = goalCalculator.getTurretAngleToGoal(follower.getPose());
            turretControl.setTargetAngle(goalAngle);
        } else {
            // When parking (state 9 or 10), set turret heading to 90 degrees.
            turretControl.setTargetAngle(90.0);
        }

        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(paths.preloadshootrow1approach, true);
                    pathState = 1;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.row1approw1capture, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.row1captureshootrow1, true);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootrow1row2approach, true);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.row2approw2capture, true);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.row2capturerow2shoot, true);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.row2shootrow3approach, true);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.row3approw3capture, true);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.row3capturepark, true);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    telemetry.addLine("Path Chain Complete");
                    pathState = 10; // End state
                }
                break;
            case 10:
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
                            new Pose(51.000, 93.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-150))
                    .setReversed(true)
                    .build();

            preloadshootrow1approach = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(51.000, 93.000),
                            new Pose(47.988, 84.012),
                            new Pose(42.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(180))
                    .build();

            row1approw1capture = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(42.000, 84.000),
                            new Pose(15.000, 84.000)))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();

            row1captureshootrow1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(15.000, 84.000),
                            new Pose(51.000, 93.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-170))
                    .setReversed(true)
                    .build();

            shootrow1row2approach = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(51.000, 93.000),
                            new Pose(51.137, 69.765),
                            new Pose(45.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-170), Math.toRadians(180))
                    .build();

            row2approw2capture = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(45.000, 60.000),
                            new Pose(15.000, 60.000)))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();

            row2capturerow2shoot = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(15.000, 60.000),
                            new Pose(42.000, 72.000),
                            new Pose(51.000, 93.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-150))
                    .build();

            row2shootrow3approach = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(51.000, 93.000),
                            new Pose(47.855, 63.129),
                            new Pose(45.000, 36.000)))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();

            row3approw3capture = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(45.000, 36.000),
                            new Pose(18.000, 36.000)))
                    .setTangentHeadingInterpolation()
                    .setReversed(true)
                    .build();

            row3capturepark = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(18.000, 36.000),
                            new Pose(42.000, 66.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}
