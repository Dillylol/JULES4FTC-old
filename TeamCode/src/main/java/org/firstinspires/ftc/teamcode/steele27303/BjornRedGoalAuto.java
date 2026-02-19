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

@Autonomous(name = "Bjorn Red Goal Auto", group = "Red")
public class BjornRedGoalAuto extends OpMode {

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
        // Start pose based on Path1 start: (111, 138) with heading 0 (implied by path1 interpolation start)
        follower.setStartingPose(new Pose(111.000, 138.000, Math.toRadians(0)));
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
        
        // --- Initialize Goal Calculator for RED Alliance ---
        goalCalculator = new GoalCalculator(
            BjornConstants.FieldPositions.RED_SCORING_X,
            BjornConstants.FieldPositions.RED_SCORING_Y
        );
        
        telemetry.addLine("Bjorn Red Goal Auto Initialized");
        telemetry.update();
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
        
        // --- Turret Logic ---
        if (pathState < 10) {
            updateTurretLogic();
        } else {
            // Path 10 starts or has finished -> Aim at 0
            turretControl.setTargetAngle(0.0);
            turretControl.update(dt);
        }

        // --- Path State Machine ---
        autonomousPathUpdate();

        // --- Telemetry ---
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret Target", turretControl.getTargetAngle());
        telemetry.update();
    }
    
    private void updateTurretLogic() {
        double goalAngle = goalCalculator.getTurretAngleToGoal(follower.getPose());
        turretControl.setTargetAngle(goalAngle);
        turretControl.update(dt);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    // Done
                    telemetry.addLine("Path Chain Complete");
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        telemetry.addData("Path State", pathState);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(111.000, 138.000),

                            new Pose(93.000, 93.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
            .setReversed()
            .build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(93.000, 93.000),
                            new Pose(96.000, 84.000),
                            new Pose(102.000, 84.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

            .build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(102.000, 84.000),

                            new Pose(132.000, 84.000)
                    )
            ).setTangentHeadingInterpolation()

            .build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(132.000, 84.000),

                            new Pose(93.000, 93.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(350))
            .setReversed()
            .build();

            Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(93.000, 93.000),
                            new Pose(87.000, 63.000),
                            new Pose(99.000, 60.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(0))

            .build();

            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(99.000, 60.000),

                            new Pose(132.000, 60.000)
                    )
            ).setTangentHeadingInterpolation()

            .build();

            Path7 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(132.000, 60.000),
                            new Pose(102.000, 72.000),
                            new Pose(93.000, 93.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(330))

            .build();

            Path8 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(93.000, 93.000),
                            new Pose(84.000, 48.000),
                            new Pose(99.000, 36.000)
                    )
            ).setTangentHeadingInterpolation()

            .build();

            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(99.000, 36.000),

                            new Pose(126.000, 36.000)
                    )
            ).setTangentHeadingInterpolation()

            .build();

            Path10 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(126.000, 36.000),

                            new Pose(96.000, 72.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

            .build();
        }
    }
}
