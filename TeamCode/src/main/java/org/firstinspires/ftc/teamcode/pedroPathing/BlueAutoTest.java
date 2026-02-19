package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.PedroPathingSetup;

@Autonomous(name = "BlueAutoTest")
@Configurable
public class BlueAutoTest extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = PedroPathingSetup.createFollower(hardwareMap);
        // Constants.setConstants(FConstants.class, LConstants.class); // Handled by PedroPathingSetup

        paths = new Paths(follower);

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
                    follower.followPath(paths.path2ScorePickup1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.path3Pickup1Score);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.path4ScorePickup2);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.path5Pickup2Score);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.path6ScorePickup3);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.path7Pickup3Score);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1); // End state
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        panelsTelemetry.debug("Path State", pathState);
    }

    public static class Paths {
        public PathChain path1StartScorePreload;
        public PathChain path2ScorePickup1;
        public PathChain path3Pickup1Score;
        public PathChain path4ScorePickup2;
        public PathChain path5Pickup2Score;
        public PathChain path6ScorePickup3;
        public PathChain path7Pickup3Score;

        public Paths(Follower follower) {
            path1StartScorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(0.000, 0.000),
                    new Pose(15.000, -33.000),
                    new Pose(21.000, -51.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-135))
                .build();

            path2ScorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(21.000, -51.000),
                    new Pose(6.000, -51.000),
                    new Pose(-18.000, -54.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(180))
                .build();

            path3Pickup1Score = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Pose(-18.000, -54.000),
                    new Pose(21.000, -51.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-135))
                .build();

            path4ScorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(21.000, -51.000),
                    new Pose(20.000, -81.000),
                    new Pose(-18.000, -78.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(180))
                .build();

            path5Pickup2Score = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Pose(-18.000, -78.000),
                    new Pose(21.000, -51.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-135))
                .build();

            path6ScorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(21.000, -51.000),
                    new Pose(14.953, -116.977),
                    new Pose(-18.023, -101.000),
                    new Pose(-22.000, -103.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(180))
                .build();

            path7Pickup3Score = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Pose(-22.000, -103.000),
                    new Pose(15.117, -89.942)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        }
    }
}
