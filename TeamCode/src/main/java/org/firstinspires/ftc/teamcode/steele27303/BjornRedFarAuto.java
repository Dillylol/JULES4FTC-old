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
// import org.firstinspires.ftc.teamcode.common.turret.GoalCalculator; // Not used in this specific logic yet, but good to have if needed later

@Autonomous(name = "Bjorn Red Far Auto", group = "Red")
public class BjornRedFarAuto extends OpMode {

    private Follower follower;
    private Paths paths;
    private int pathState = 0;

    private TurretControl turretControl;
    private IMU imu;
    private DcMotorEx turretMotor;

    private double lastLoopTime = 0.0;
    private double dt = 0.02;

    @Override
    public void init() {
        // --- Initialize Pedro Follower ---
        follower = Constants.createFollower(hardwareMap);
        // Start pose based on 'gotopreloadzoneshoot' start: (90, 9) with heading 90
        follower.setStartingPose(new Pose(90.000, 9.000, Math.toRadians(90)));
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

        telemetry.addLine("Bjorn Red Far Auto Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start the first path immediately
        follower.followPath(paths.gotopreloadzoneshoot, true);
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
        
        // Update Turret Control Loop
        turretControl.update(dt);

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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Currently following gotopreloadzoneshoot
                if (!follower.isBusy()) {
                    follower.followPath(paths.preloadtofirstcatch, true);
                    pathState = 1;
                }
                break;
            case 1:
                // Currently following preloadtofirstcatch
                if (!follower.isBusy()) {
                    follower.followPath(paths.gotoshoot2, true);
                    pathState = 2;
                }
                break;
            case 2:
                // Currently following gotoshoot2
                if (!follower.isBusy()) {
                    // USER REQUEST: when path 7 starts, set turret heading to 0
                    turretControl.setTargetAngle(0); 
                    
                    follower.followPath(paths.Path7, true);
                    pathState = 3;
                }
                break;
            case 3:
                // Currently following Path7
                if (!follower.isBusy()) {
                    // Done
                    telemetry.addLine("Path Chain Complete");
                    pathState = 4; // End state
                }
                break;
            case 4:
                // End state - do nothing
                break;
        }
    }

    public static class Paths {
        public PathChain gotopreloadzoneshoot;
        public PathChain preloadtofirstcatch;
        public PathChain gotoshoot2;
        public PathChain Path7;

        public Paths(Follower follower) {
            gotopreloadzoneshoot = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(90.000, 9.000),

                            new Pose(72.000, 72.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(105))

            .build();

            preloadtofirstcatch = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(72.000, 72.000),
                            new Pose(99.000, 36.000),
                            new Pose(129.000, 36.000)
                    )
            ).setTangentHeadingInterpolation()

            .build();

            gotoshoot2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(129.000, 36.000),

                            new Pose(72.000, 72.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(105))
            .setReversed()
            .build();

            Path7 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(72.000, 72.000),

                            new Pose(81.000, 36.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(0))

            .build();
        }
    }
}
