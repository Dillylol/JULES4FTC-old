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

@Autonomous(name = "RedAutoTest")
@Configurable
public class RedAutoTest extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = PedroPathingSetup.createFollower(hardwareMap);
        // Constants.setConstants(FConstants.class, LConstants.class);

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
                    new Pose(110.500, 135.500),
                    new Pose(96.000, 105.000),
                    new Pose(90.000, 87.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
                .build();

            path2ScorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(90.000, 87.000),
                    new Pose(105.000, 87.000),
                    new Pose(129.000, 84.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .build();

            path3Pickup1Score = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Pose(129.000, 84.000),
                    new Pose(90.000, 87.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .build();

            path4ScorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(90.000, 87.000),
                    new Pose(91.000, 57.000),
                    new Pose(129.000, 60.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .build();

            path5Pickup2Score = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Pose(129.000, 60.000),
                    new Pose(90.000, 87.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .build();

            path6ScorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(90.000, 87.000),
                    new Pose(96.047, 21.023),
                    new Pose(129.023, 37.000),
                    new Pose(133.000, 35.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .build();

            path7Pickup3Score = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Pose(133.000, 35.000),
                    new Pose(95.883, 48.058)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        }
    }
}
