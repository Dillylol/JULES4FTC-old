package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Bjorn Blue Line Auto", group = "Test")
public class BjornBlueLineAuto extends OpMode {

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
        // Setting start pose to match the start of Path1 (72, 72) with 90 deg heading
        follower.setStartingPose(new Pose(72.000, 72.000, Math.toRadians(90)));
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
        
        // --- Initialize Goal Calculator ---
        goalCalculator = new GoalCalculator(
            BjornConstants.FieldPositions.BLUE_SCORING_X,
            BjornConstants.FieldPositions.BLUE_SCORING_Y
        );
        
        telemetry.addLine("Bjorn Blue Line Auto Initialized");
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
        
        // Basic Turret Logic (Home to 0 or something safe for this test)
        turretControl.setTargetAngle(0.0);
        turretControl.update(dt);

        // --- Path State Machine ---
        autonomousPathUpdate();

        // --- Telemetry ---
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    setPathState(2); 
                    telemetry.addLine("Path 1 Complete");
                }
                break;
            case 2:
                // End state
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        telemetry.addData("Path State", pathState);
    }

    public static class Paths {
        public PathChain Path1;
        
        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(72.000, 72.000),
                    new Pose(72.000, 111.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            .build();
        }
    }
}
