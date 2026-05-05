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
import org.firstinspires.ftc.teamcode.common.turret.TurretControl;

@Autonomous(name = "farParkOnlyAutoBlue")
public class FarParkOnlyAutoBlue extends OpMode {

    private Follower follower;
    private Paths paths;
    private int pathState = 0;

    private TurretControl turretControl;
    private IMU imu;
    private DcMotorEx turretMotor;

    private BjornHardware hardwareWrapper;

    private double lastLoopTime = 0.0;
    private double dt = 0.02;

    @Override
    public void init() {
        // --- Initialize Pedro Follower ---
        follower = Constants.createFollower(hardwareMap);
        // Start pose based on the first path start
        follower.setStartingPose(new Pose(57.000, 6.000, Math.toRadians(180)));
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
        
        // --- Initialize Hardware ---
        hardwareWrapper = BjornHardware.forTeleOp(hardwareMap);

        telemetry.addLine("Far Park Only Auto Blue Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(paths.startpark, true);
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

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret Target", turretControl.getTargetAngle());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        // Just keep the turret tucked at 90 degrees local while parking
        turretControl.setTargetMode(TurretControl.TargetMode.LOCAL);
        turretControl.setTargetAngle(90.0);

        long nowMs = System.currentTimeMillis();

        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    telemetry.addLine("Path Chain Complete");
                    pathState = 1; // End state
                }
                break;
            case 1:
                break;
        }
    }

    public static class Paths {
        public PathChain startpark;

        public Paths(Follower follower) {
            startpark = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(57.000, 6.000),
                            new Pose(57.000, 60.000),
                            new Pose(45.000, 42.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}
