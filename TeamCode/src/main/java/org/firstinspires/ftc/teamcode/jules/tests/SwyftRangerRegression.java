package org.firstinspires.ftc.teamcode.jules.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.SwyftRangerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Swyft Ranger Regression", group = "Test")
public class SwyftRangerRegression extends LinearOpMode {

    private Follower follower;
    private BjornHardware robot;

    // Regression Data Points
    private final List<Double> voltages = new ArrayList<>();
    private final List<Double> distancesInches = new ArrayList<>();

    @Override
    public void runOpMode() {
        // Initialize Hardware
        robot = BjornHardware.forTeleOp(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        // Assume start at (0,0) facing 0 degrees (Forward direction along X)
        // Reversing implies moving in -X direction.
        Pose startPose = new Pose(0, 0, Math.toRadians(0));
        follower.setStartingPose(startPose);

        telemetry.addLine("Initialized Swyft Ranger Regression");
        telemetry.addLine("Setup: Front of robot against wall (or known zero).");
        telemetry.addLine("Robot will REVERSE (move -X) from 1ft to 10ft.");
        telemetry.addLine("Ensure clear path behind robot.");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();

        for (int i = 1; i <= 10; i++) {
            if (isStopRequested()) return;

            // Target X: -1 foot * i (Moving backwards along X)
            double targetDistInches = i * 12.0;
            Pose targetPose = new Pose(-targetDistInches, 0, Math.toRadians(0));

            telemetry.addData("Status", "Moving to %d ft...", i);
            telemetry.update();

            // Create Path
            Path path = new Path(new BezierLine(follower.getPose(), targetPose));
            path.setConstantHeadingInterpolation(0);
            
            follower.followPath(path, true); // holdEnd = true

            // Wait until reached
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                telemetry.addData("Moving to", targetDistInches + " inches");
                telemetry.addData("Current X", follower.getPose().getX());
                telemetry.update();
            }
            
            // Extra settle for potential overshoot/oscillation
            long stopTime = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - stopTime < 1000) {
                 follower.update();
            }

            // Collect samples
            telemetry.addData("Status", "Sampling at %d ft...", i);
            telemetry.update();

            for (int k = 0; k < 10; k++) {
                if (isStopRequested()) break;
                
                follower.update(); // Keep keeping position
                
                double volts = robot.swyftRanger.getVoltage();
                // Odo distance is absolute X distance from start (0)
                double odoDist = Math.abs(follower.getPose().getX());

                voltages.add(volts);
                distancesInches.add(odoDist);

                telemetry.addData("Sample", "%d/10", k+1);
                telemetry.addData("Volts", "%.3f", volts);
                telemetry.addData("Odo", "%.2f", odoDist);
                telemetry.update();
                sleep(100);
            }
        }

        // Compute Regression
        // y = mx + b -> Distance = m * Voltage + b
        int n = voltages.size();
        double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;

        for (int i = 0; i < n; i++) {
            double x = voltages.get(i);
            double y = distancesInches.get(i);
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
        }

        double m = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
        double b = (sumY - m * sumX) / n;

        // Display Results
        while (opModeIsActive()) {
            telemetry.addLine("=== REGRESSION RESULTS ===");
            telemetry.addData("Data Points", n);
            telemetry.addData("Slope (m)", "%.4f", m);
            telemetry.addData("Intercept (b)", "%.4f", b);
            telemetry.addLine("Formula: Dist(in) = " + String.format("%.2f", m) + " * Volts + " + String.format("%.2f", b));
            telemetry.addLine("\n=== COMPARISON (Last Sample) ===");
            
            if (n > 0) {
                 double lastV = voltages.get(n-1);
                 double lastOdo = distancesInches.get(n-1);
                 double calcDist = m * lastV + b;
                 double constDist = SwyftRangerConstants.voltageToInches(lastV);
                 
                 telemetry.addData("Input Volts", "%.3f", lastV);
                 telemetry.addData("Odometry", "%.2f in", lastOdo);
                 telemetry.addData("Regression Calc", "%.2f in", calcDist);
                 telemetry.addData("Old Constant Calc", "%.2f in", constDist);
            }
            
            telemetry.update();
        }
    }
}
