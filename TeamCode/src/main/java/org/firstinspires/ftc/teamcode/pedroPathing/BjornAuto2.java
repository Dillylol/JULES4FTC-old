package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.shooter.AutoShooter;
import org.firstinspires.ftc.teamcode.common.turret.TurretControl;
import org.firstinspires.ftc.teamcode.configurables.ShooterConfigurables;

/**
 * BjornAuto2 - New Autonomous Pathing
 * Headings are adjusted by 180 degrees to match robot coordinate system.
 * (User Input 90 -> Code 270, User Input 180 -> Code 0)
 */
@Autonomous(name = "BjornAuto2", group = "Autonomous")
@Configurable
public class BjornAuto2 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private BjornHardware hardware;
    private AutoShooter shooter;
    private long shootTimer = -1;
    private boolean intakeLatched = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Start Pose: (33.868, 135.471). Heading: 90 -> Adjusted to 270 (Math.toRadians(270))
        follower.setStartingPose(new Pose(33.868, 135.471, Math.toRadians(270)));

        paths = new Paths(follower);
        
        hardware = BjornHardware.forAutonomous(hardwareMap);
        shooter = new AutoShooter(hardware);
        
        if (hardware.turret != null) {
            hardware.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.turret.setPower(0);
        }

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.StartScore90deg);
                setPathState(10); // Go to Shoot 1
                break;
            
            case 10: // Shoot 1 (Preload)
                if (!follower.isBusy()) {
                    startShooting();
                    setPathState(11);
                }
                break;
            case 11:
                if (runShootingLoop()) {
                    setPathState(1); // Proceed to Sample 1
                }
                break;

            case 1: // Score -> Set 1 Approach
                follower.followPath(paths.Scoreset1approach);
                if (hardware.intake != null) hardware.intake.setPower(0.7); // Intake Transit
                setPathState(2);
                break;
            
            case 2: // Set 1 Approach -> Set 1 Capture
                if (!follower.isBusy()) {
                    follower.followPath(paths.set1approachset1capture);
                    if (hardware.intake != null) hardware.intake.setPower(1.0); // Intake Capture
                    setPathState(3);
                }
                break;

            case 3: // Capture -> Score
                if (!follower.isBusy()) {
                    if (hardware.intake != null) hardware.intake.setPower(0.0); // Stop Intake
                    follower.followPath(paths.set1capturescore2180deg);
                    setPathState(30); // Go to Shoot 2
                }
                break;

            case 30: // Shoot 2
                if (!follower.isBusy()) {
                    startShooting();
                    setPathState(31);
                }
                break;
            case 31: 
                if (runShootingLoop()) {
                    setPathState(4); // Proceed to Set 2
                }
                break;

            case 4: // Score -> Set 2 Approach
                follower.followPath(paths.Score2set2approach);
                if (hardware.intake != null) hardware.intake.setPower(0.7);
                setPathState(5);
                break;

            case 5: // Set 2 Approach -> Set 2 Capture
                if (!follower.isBusy()) {
                    follower.followPath(paths.set2approachSet2capture);
                    if (hardware.intake != null) hardware.intake.setPower(1.0);
                    setPathState(6);
                }
                break;
            
            case 6: // Capture -> Score
                if (!follower.isBusy()) {
                    if (hardware.intake != null) hardware.intake.setPower(0.0);
                    follower.followPath(paths.set2capturescore390);
                    setPathState(60); // Go to Shoot 3
                }
                break;

            case 60: // Shoot 3
                if (!follower.isBusy()) {
                    startShooting();
                    setPathState(61);
                }
                break;
            case 61:
                if (runShootingLoop()) {
                    setPathState(7); // Proceed to Set 3
                }
                break;

            case 7: // Score -> Set 3 Approach
                follower.followPath(paths.Score3Set3approach);
                if (hardware.intake != null) hardware.intake.setPower(0.7);
                setPathState(8);
                break;
            
            case 8: // Set 3 Approach -> Set 3 Capture
                if (!follower.isBusy()) {
                     follower.followPath(paths.set3approachset3capture);
                     if (hardware.intake != null) hardware.intake.setPower(1.0);
                     setPathState(9);
                }
                break;

            case 9: // Capture -> Park
                if (!follower.isBusy()) {
                    if (hardware.intake != null) hardware.intake.setPower(0.0);
                    follower.followPath(paths.set3capturepark);
                    setPathState(-1); // End
                }
                break;
        }
    }

    private void startShooting() {
        shootTimer = -1;
        // Fixed distance calc or reuse same logic?
        // Using same logic as RawPedro
        double distFeet = 37.0 / 12.0;
        int targetWithOffset = (int) (ShooterConfigurables.rpmSlopeRanger * distFeet 
                                    + ShooterConfigurables.rpmOffsetRanger);
        shooter.setTargetRpm(targetWithOffset + 200);
    }

    private boolean runShootingLoop() {
        shooter.update();
        boolean ready = shooter.isReady();
        
        if (ready) {
            intakeLatched = true;
            if (shootTimer == -1) shootTimer = System.currentTimeMillis();
        }
        
        if (intakeLatched) {
             if (hardware.intake != null) hardware.intake.setPower(1.0);
        } else {
             if (hardware.intake != null) hardware.intake.setPower(0);
        }

        if (shootTimer != -1 && (System.currentTimeMillis() - shootTimer > 3000)) { // Reduced to 3s for speed? Or keep 6s? RawPedro had 6s.
            // Let's use 3s to make it flow faster since there are more cycles
            shooter.setTargetRpm(0);
            shooter.update();
            if (hardware.intake != null) hardware.intake.setPower(0);
            intakeLatched = false;
            return true;
        }
        return false;
    }

    public void setPathState(int pState) {
        pathState = pState;
        panelsTelemetry.debug("Path State", pathState);
    }

    public static class Paths {
        public PathChain StartScore90deg;
        public PathChain Scoreset1approach;
        public PathChain set1approachset1capture;
        public PathChain set1capturescore2180deg;
        public PathChain Score2set2approach;
        public PathChain set2approachSet2capture;
        public PathChain set2capturescore390;
        public PathChain Score3Set3approach;
        public PathChain set3approachset3capture;
        public PathChain set3capturepark;

        public Paths(Follower follower) {
            // NOTE: Headings adjusted by 180 (User 90 -> Code 270, User 180 -> Code 0)
            
            StartScore90deg = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(33.868, 135.471),
                    new Pose(39.000, 105.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(270)) // 90 -> 270
                .build();

            Scoreset1approach = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(39.000, 105.000),
                    new Pose(45.000, 84.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0)) // 90->270, 180->0
                .build();

            set1approachset1capture = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(45.000, 84.000),
                    new Pose(15.000, 84.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0)) // 180 -> 0
                .build();

            set1capturescore2180deg = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(15.000, 84.000),
                    new Pose(39.000, 105.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270)) // 180->0, 90->270
                .build();

            Score2set2approach = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(39.000, 105.000),
                    new Pose(45.000, 75.000),
                    new Pose(42.000, 60.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0)) // 90->270, 180->0
                .setReversed()
                .build();

            set2approachSet2capture = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(42.000, 60.000),
                    new Pose(15.000, 60.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0)) // 180 -> 0
                .build();

            set2capturescore390 = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(15.000, 60.000),
                    new Pose(15.000, 75.000),
                    new Pose(39.000, 105.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270)) // 180->0, 90->270
                .build();

            Score3Set3approach = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(39.000, 105.000),
                    new Pose(54.000, 36.000),
                    new Pose(42.000, 36.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0)) // 90->270, 180->0
                .build();

            set3approachset3capture = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(42.000, 36.000),
                    new Pose(15.000, 36.000)
                ))
                .setTangentHeadingInterpolation()
                .build();

            set3capturepark = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(15.000, 36.000),
                    new Pose(66.000, 36.000)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        }
    }
}
