package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.SwyftRangerConstants;
import org.firstinspires.ftc.teamcode.common.shooter.TeleOpShooter;

import java.util.ArrayList;
import java.util.List;

/**
 * Regression Shooter Auto
 * 
 * 1. Drives to specific distance spot (starting 10 inches).
 * 2. Allows user to shoot (TeleOp controls).
 * 3. Uses Regression to set RPM based on Ranger distance.
 * 4. Prompts for feedback (Hit/Miss).
 * 5. Updates Regression model.
 * 6. Moves to next distance (+10 inches), loops at 100.
 */
@TeleOp(name = "RegressionShooterAuto", group = "Test")
public class RegressionShooterAuto extends OpMode {

    // ---------------- Hardware ----------------
    private Follower follower;
    private BjornHardware hardware;
    private TeleOpShooter shooter;
    
    // ---------------- Poses ----------------
    // We assume shooting "backwards" effectively since the shooter is usually on the back?
    // Or we align the robot specific way. 
    // Let's define a 'Goal' point and we move along a vector from it.
    // Based on BjornAutoBLUE, SHOOT_ZONE is (0, 25).
    // Let's assume we want to move along the Y axis or a specific line.
    // Ideally user would calibrate this, but I'll set a standard line.
    // Let's use (0,y) where y increases as distance increases.
    
    // START Pose
    private static final Pose START_POSE = new Pose(0, 0, Math.toRadians(265)); // Matching BjornAutoBlue
    
    // ---------------- State Machine ----------------
    private enum State {
        INIT,
        DRIVING,
        TELEOP_AIM,
        SHOOTING_DETECT, // Waiting for shot to finish
        FEEDBACK_PROMPT,
        CALCULATE_NEXT
    }

    private State state = State.INIT;
    private double currentTargetDistanceInches = 10.0;
    
    // ---------------- Regression ----------------
    private LinearRegression regression = new LinearRegression();
    
    // ---------------- Input Tracking ----------------
    private boolean rtPrev = false;
    private boolean xPrev = false;
    private boolean bPrev = false;
    private boolean aPrev = false; // gamepad1.a
    private boolean x1Prev = false; // gamepad1.x

    // ---------------- Telemetry/Data ----------------
    private double lastShotRpm = 0;
    private double lastShotDist = 0;
    private int successCount = 0;
    private ElapsedTime promptTimer = new ElapsedTime();


    @Override
    public void init() {
        // Initialize Pedro Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        // Initialize Hardware
        hardware = BjornHardware.forTeleOp(hardwareMap); // Use forTeleOp configuration for the controls
        shooter = new TeleOpShooter(hardware);

        // Seed the regression with some safe defaults if needed?
        // Or start fresh. Let's start fresh but provide a safe fallback in code.
        // Adding 2 initial points to avoid div by zero and give a baseline slope
        // Point 1: 10 inches -> ~2200 RPM (Estimated)
        // Point 2: 100 inches -> ~3500 RPM (Estimated)
        // These are just guesses to prevent NaN, model will adjust.
        regression.addPoint(12.0, 2200.0);
        regression.addPoint(72.0, 3000.0); 

        telemetry.addLine("Regression Auto Initialized");
        telemetry.addLine("Press START.");
    }

    @Override
    public void start() {
        // Generate first path
        generatePathToDistance(currentTargetDistanceInches);
        state = State.DRIVING;
    }

    @Override
    public void loop() {
        follower.update();
        hardware.updateEstimator(); // Keep turret estimator happy if used
        shooter.update(); // Update shooter logic (readiness, etc)

        switch (state) {
            case DRIVING:
                if (!follower.isBusy() || follower.atParametricEnd()) {
                    // Arrived
                    state = State.TELEOP_AIM;
                }
                break;

            case TELEOP_AIM:
                handleTeleOpControls();
                
                // Auto-Calculate RPM based on Range
                double rangeDistAndVolts[] = getRangerDistance();
                double dist = rangeDistAndVolts[0];
                double volts = rangeDistAndVolts[1];
                
                double targetRpm = regression.predict(dist);
                // Clamp RPM
                targetRpm = Math.max(1000, Math.min(4500, targetRpm));
                
                // Set Shooter Target
                shooter.setTargetRpm((int)targetRpm);
                
                telemetry.addData("Mode", "AIM & SHOOT");
                telemetry.addData("Distance", "%.1f in (%.2f V)", dist, volts);
                telemetry.addData("Target RPM", "%.0f", targetRpm);
                
                // Detect Shot (Right Trigger)
                if (gamepad1.right_trigger > 0.5) {
                    lastShotRpm = targetRpm;
                    lastShotDist = dist;
                    state = State.SHOOTING_DETECT;
                }
                break;
                
            case SHOOTING_DETECT:
                handleTeleOpControls(); // Allow follow through
                // Wait for trigger release
                if (gamepad1.right_trigger < 0.1) {
                    state = State.FEEDBACK_PROMPT;
                    promptTimer.reset();
                    // Stop shooter/intake for safety during prompt? 
                    // User might want to keep spinning, but let's stop to be clear.
                    // shooter.setTargetRpm(0); // Optional: Stop flywheel? leaving it running for faster cycle
                }
                break;

            case FEEDBACK_PROMPT:
                // Stop base movement if any? (Already stopped)
                
                telemetry.addData("Mode", "FEEDBACK");
                telemetry.addData("Last Shot", "%.1f in @ %.0f RPM", lastShotDist, lastShotRpm);
                telemetry.addLine("Did it HIT?");
                telemetry.addLine("Press X for YES");
                telemetry.addLine("Press B for NO");
                
                if (gamepad1.x && !xPrev) {
                    // YES
                    regression.addPoint(lastShotDist, lastShotRpm);
                    successCount++;
                    telemetry.addLine("Recorded SUCCESS!");
                    telemetry.update();
                    state = State.CALCULATE_NEXT;
                } else if (gamepad1.b && !bPrev) {
                    // NO
                    telemetry.addLine("Recorded MISS (Ignored)");
                    telemetry.update();
                    state = State.CALCULATE_NEXT;
                }
                break;

            case CALCULATE_NEXT:
                // Increment Distance
                currentTargetDistanceInches += 10.0;
                if (currentTargetDistanceInches > 100.0) {
                    currentTargetDistanceInches = 10.0;
                }
                
                generatePathToDistance(currentTargetDistanceInches);
                state = State.DRIVING;
                break;
        }
        
        // Always show regression status
        telemetry.addData("State", state);
        telemetry.addData("Distance Target", "%.1f in", currentTargetDistanceInches);
        telemetry.addData("Success Count", successCount);
        telemetry.addData("Model Slope", "%.4f", regression.slope);
        telemetry.addData("Model Intercept", "%.4f", regression.intercept);
        
        updateInputTracking();
    }

    // ---------------- Helper Methods ----------------

    private void generatePathToDistance(double distInches) {
        // Create Path to target distance
        // We use Pose objects instead of Points as BezierLine accepts Poses.
        // Facing 270 degrees (down/backwards relative to start?)
        Pose currentPose = follower.getPose();
        Pose targetPose = new Pose(0, distInches, Math.toRadians(270)); 
        
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading()), targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();
                
        follower.followPath(path, true);
    }

    private double[] getRangerDistance() {
        // Read voltage
        if (hardware.swyftRanger == null) return new double[]{0,0};
        double volts = hardware.swyftRanger.getVoltage();
        double batVal = hardware.getBatteryVoltage();
        double dist = SwyftRangerConstants.voltageToInches(volts, batVal);
        return new double[]{dist, volts};
    }

    private void handleTeleOpControls() {
        // Intake (A / X)
        double intakePower = 0.0;
        if (gamepad1.a) intakePower = 1.0;
        else if (gamepad1.x) intakePower = -1.0;
        hardware.intake.setPower(intakePower);

        // Grip (RT = Shoot/Intake, LT = Outtake)
        // User said: "right trigger for grip servos"
        // BjornTeleBase: RT -> Grip=1.0. 
        if (gamepad1.right_trigger > 0.5) {
            hardware.grip1.setPower(1.0);
            hardware.grip2.setPower(1.0);
        } else if (gamepad1.left_trigger > 0.5) {
            hardware.grip1.setPower(-1.0);
            hardware.grip2.setPower(-1.0);
        } else {
            hardware.grip1.setPower(0.0);
            hardware.grip2.setPower(0.0);
        }
        
        // Flywheel toggles (optional override)
        // Shooter handles its own loop logic, but we are setting target RPM manually
        // so we don't need to toggle it on/off with buttons unless desired.
    }

    private void updateInputTracking() {
        rtPrev = gamepad1.right_trigger > 0.5;
        xPrev = gamepad1.x;
        bPrev = gamepad1.b;
        aPrev = gamepad1.a;
    }

    // ---------------- Linear Regression Class ----------------
    private static class LinearRegression {
        private double sumX = 0;
        private double sumY = 0;
        private double sumXY = 0;
        private double sumX2 = 0;
        private int n = 0;
        
        public double slope = 0;
        public double intercept = 0;

        public void addPoint(double x, double y) {
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
            n++;
            recalculate();
        }

        private void recalculate() {
            if (n < 2) return;
            double denominator = n * sumX2 - sumX * sumX;
            if (denominator == 0) return; // Avoid division by zero
            
            slope = (n * sumXY - sumX * sumY) / denominator;
            intercept = (sumY - slope * sumX) / n;
        }

        public double predict(double x) {
            if (n < 2) return 2500; // Default if not enough data
            return slope * x + intercept;
        }
    }
}
