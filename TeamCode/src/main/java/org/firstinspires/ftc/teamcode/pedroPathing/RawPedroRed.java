package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.turret.TurretControl;
import org.firstinspires.ftc.teamcode.configurables.ShooterConfigurables;
import org.firstinspires.ftc.teamcode.common.shooter.AutoShooter;

@Autonomous(name = "Raw Pedro Red", group = "Pedro")
@Configurable // Panels
public class RawPedroRed extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private BjornHardware hardware;
    private AutoShooter shooter;
    private long shootTimer = -1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Setting start pose to match the first path's start point: (33, 6, 180 deg)
        // Mirrored Y: 144 - 138 = 6
        follower.setStartingPose(new Pose(33.000, 6.000, Math.toRadians(0)));

        paths = new Paths(follower); // Build paths
        
        // Initialize Hardware using BjornHardware
        hardware = BjornHardware.forAutonomous(hardwareMap);
        shooter = new AutoShooter(hardware);
        
        // Lock Turret
        if (hardware.turret != null) {
            hardware.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.turret.setPower(0);
        }

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.path1StartScorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    startShooting();
                    setPathState(100); // Shoot 1 (Preload)
                }
                break;
            case 100: // Shooting Phase 1
                if (runShootingLoop()) {
                   follower.followPath(paths.path2ScorePoint3);
                   if (hardware.intake != null) hardware.intake.setPower(1.0); // Intake ON (Transit/Prep? User said full speed)
                   // The prompt said "intake is set to full speed" generally? 
                   // Or during shooting? "fired over a period of 10 seconds and intake is set to full speed"
                   // implies full speed during firing.
                   // I'll set it to 0.7 for transit (standard) and 1.0 for shooting.
                   if (hardware.intake != null) hardware.intake.setPower(0.7); 
                   setPathState(2);
                }
                break;

            case 2: // Path 2: Score -> Point 3
                if (!follower.isBusy()) {
                    follower.followPath(paths.path3ScorePickup1);
                    setPathState(3);
                }
                break;
            case 3: // Path 3: Point 3 -> Pickup 1
                if (!follower.isBusy()) {
                    if (hardware.intake != null) hardware.intake.setPower(0); // Intake OFF before grabbing?
                    // Usually we turn intake OFF to grab, or ON if using active intake.
                    // Assuming similar logic to before: OFF to ready for next move or grab.
                    follower.followPath(paths.pathPickup1Score);
                    setPathState(4);
                }
                break;
            case 4: // Pickup 1 -> Score
                if (!follower.isBusy()) {
                    startShooting();
                    setPathState(400); // Shoot 2
                }
                break;
            case 400: // Shooting Phase 2
                if (runShootingLoop()) {
                    follower.followPath(paths.path4ScorePickup2);
                    if (hardware.intake != null) hardware.intake.setPower(0.7); // Intake ON for transit
                    setPathState(5);
                }
                break;

            case 5: // Score -> Pickup 2 
                if (!follower.isBusy()) {
                    follower.followPath(paths.path5Pickup2Score);
                    setPathState(6);
                }
                break;
            case 6: // Pickup 2 Prep
                if (!follower.isBusy()) {
                     if (hardware.intake != null) hardware.intake.setPower(0);
                    follower.followPath(paths.path6ScorePickup3); // Moves to Park/End
                    setPathState(7);
                }
                break;
            
            case 7: // Park/End
                if (!follower.isBusy()) {
                    setPathState(-1); // End state
                }
                break;
        }
    }

    private void startShooting() {
        shootTimer = -1;
        // Calculate RPM for 37 inches
        double distFeet = 37.0 / 12.0;
        int targetWithOffset = (int) (ShooterConfigurables.rpmSlopeRanger * distFeet 
                                    + ShooterConfigurables.rpmOffsetRanger);
        
        // Add manual offset +200 RPM
        shooter.setTargetRpm(targetWithOffset + 200);
    }

    private boolean intakeLatched = false;

    private boolean runShootingLoop() {
        shooter.update();

        boolean ready = shooter.isReady();
        
        // Intake logic: Latch ON (1.0) once ready and stay on
        if (ready) {
            intakeLatched = true;
            // Start timer when first ready
            if (shootTimer == -1) {
                shootTimer = System.currentTimeMillis();
            }
        }
        
        if (intakeLatched) {
             if (hardware.intake != null) hardware.intake.setPower(1.0);
        } else {
             if (hardware.intake != null) hardware.intake.setPower(0);
        }

        // Check timer - 10 second window (10000ms) AFTER becoming ready
        if (shootTimer != -1 && (System.currentTimeMillis() - shootTimer > 6000)) {
            // Done
            shooter.setTargetRpm(0);
            shooter.update();
            
            if (hardware.intake != null) hardware.intake.setPower(0);
            intakeLatched = false; // Reset latch
            
            // Force grips off
            if (hardware.grip1 != null) hardware.grip1.setPower(0);
            if (hardware.grip2 != null) hardware.grip2.setPower(0);
            
            return true;
        }
        
        return false;
    }

    public void setPathState(int pState) {
        pathState = pState;
        panelsTelemetry.debug("Path State", pathState);
    }

    public static class Paths {
        public PathChain path1StartScorePreload;
        public PathChain path2ScorePoint3;
        public PathChain path3ScorePickup1;
        public PathChain pathPickup1Score;
        public PathChain path4ScorePickup2;
        public PathChain path5Pickup2Score;
        public PathChain path6ScorePickup3;

        public Paths(Follower follower) {
            // Path 1
            // (33, 138) -> (33, 6)
            // (48, 105) -> (48, 39)
            // (42, 102) -> (42, 42)
            // Headings: 180->45 becomes 180->315
            path1StartScorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(33.000, 6.000),
                    new Pose(48.000, 39.000),
                    new Pose(42.000, 42.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

            // Path 2
            // (42, 102) -> (42, 42)
            // (48, 96) -> (48, 48)
            // (39, 93) -> (39, 51)
            // Headings: 45->180 becomes 315->180
            path2ScorePoint3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(42.000, 42.000),
                    new Pose(48.000, 48.000),
                    new Pose(39.000, 51.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

            // Path 3
            // (39, 93) -> (39, 51)
            // (15, 93) -> (15, 51)
            path3ScorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Pose(39.000, 51.000),
                    new Pose(15.000, 51.000)
                ))
                .setTangentHeadingInterpolation()
                .build();

            // Path Pickup 1
            // (15, 93) -> (15, 51)
            // (57, 93) -> (57, 51)
            // (42, 102) -> (42, 42)
            // Headings: 180->225 becomes 180->135
            pathPickup1Score = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(15.000, 51.000),
                    new Pose(57.000, 51.000),
                    new Pose(42.000, 42.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .setReversed()
                .build();

            // Path 4
            // (42, 102) -> (42, 42)
            // (39, 60) -> (39, 84)
            // Headings: 225->180 becomes 135->180
            path4ScorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Pose(42.000, 42.000),
                    new Pose(39.000, 84.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .build();

            // Path 5
            // (39, 60) -> (39, 84)
            // (15, 60) -> (15, 84)
            path5Pickup2Score = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Pose(39.000, 84.000),
                    new Pose(15.000, 84.000)
                ))
                .setTangentHeadingInterpolation()
                .build();

            // Path 6
            // (15, 60) -> (15, 84)
            // (45, 60) -> (45, 84)
            // Heading: 180->0 (unchanged)
            path6ScorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine( 
                    new Pose(15.000, 84.000),
                    new Pose(45.000, 84.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        }
    }
}
