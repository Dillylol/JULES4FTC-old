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
import org.firstinspires.ftc.teamcode.common.turret.TurretEstimator;
import org.firstinspires.ftc.teamcode.common.turret.GoalCalculator;


@Autonomous(name = "Bjorn Blue Park Test", group = "Test")
public class BjornBlueParkTest extends OpMode {

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
        follower.setStartingPose(new Pose(33.000, 135.000, Math.toRadians(0)));
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
        
        // --- Save Auto Start Pose for TeleOp Triangulation ---
        // (Persistence removed in favor of hardcoded handoff)
        // Pose startPose = follower.getPose(); 
        // BjornPersistence.saveAutoStartPose(hardwareMap.appContext, startPose);

        telemetry.addLine("Bjorn Blue Park Test Initialized");
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
        
        // Track goal during driving (pathState 0 = start→approach, 1 = approach→park)
        // Home to 90° only at pathState 2 (robot has stopped, about to park)
        if (pathState <= 1) {
            updateTurretLogic();
        } else {
            // Homing to 90° - disable goal tracking
            turretControl.setTargetAngle(90.0);
            double currentAngle = turretControl.getCurrentAngle();
            
            // If at 90°, kill motor completely
            if (Math.abs(currentAngle - 90.0) < 1.0) {
                turretMotor.setPower(0.0);
            } else {
                // TurretControl.update() handles PD + sets motor power internally
                turretControl.update(dt);
            }
        }

        // --- Path State Machine ---
        autonomousPathUpdate();

        // --- Telemetry ---
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret Target", turretControl.getTargetAngle());
        telemetry.addData("Distance", turretControl.getDistanceToTarget());
        telemetry.update();
        
        // Note: LAST_TURRET_ANGLE saved at end of homing routine
    }
    

    
    private void updateTurretLogic() {
        // Use goal calculator for dynamic aiming at goal
        double goalAngle = goalCalculator.getTurretAngleToGoal(follower.getPose());
        turretControl.setTargetAngle(goalAngle);
        
        // TurretControl.update() handles PD, FF, force field, limits,
        // and applies TURRET_POWER_DIRECTION internally via setPower().
        // Do NOT set motor power again here — it would double-invert.
        turretControl.update(dt);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.startparkapproach, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.parkapproachpark, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    setPathState(3); // Begin homing
                }
                break;
            case 3:
                // Park complete - turret already at 90° from approach
                telemetry.addLine("Park Complete. Turret at 90°.");
                telemetry.update();
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        telemetry.addData("Path State", pathState);
    }

    public static class Paths {
        public PathChain startparkapproach;
        public PathChain parkapproachpark;

        public Paths(Follower follower) {
            startparkapproach = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(33.000, 135.000),
                            new Pose(33.000, 36.000)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

            parkapproachpark = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(33.000, 36.000),
                            new Pose(60.000, 36.000)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)) // Fixed: Back to constant 180 after odo fix
            .build();
        }
    }
}
