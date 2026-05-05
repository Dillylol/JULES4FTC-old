package org.firstinspires.ftc.teamcode.steele27303;


import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

/**
 * An Automatic Pedro Tuner that attempts to find acceptable Feedforward and PID values through
 * controlled tests and error evaluation.
 */
@Autonomous(name = "Pedro Auto Tuner", group = "Tuning")
public class PedroAutoTuner extends OpMode {

    private Follower follower;

    private enum State {
        WAITING_FOR_START,
        TUNE_FF,
        TUNE_P,
        TUNE_D,
        DONE
    }

    private State state = State.WAITING_FOR_START;

    // FF Variables
    private double testPower = 0.0;
    private long lastTime = 0;
    private double bestFF = 0.0;

    // PID Variables
    private double testP = 0.05;
    private double testD = 0.0;
    private double bestP = 0.05;
    private double bestD = 0.0;
    private double minError = Double.MAX_VALUE;
    private ArrayList<Double> cycleErrors = new ArrayList<>();

    private Path forwardsPath;
    private Path backwardsPath;
    private boolean movingForward = true;

    private double errorAccumulator = 0.0;
    private int errorSamples = 0;

    private double startX = 0;
    private double endX = 48; // Sweep back and forth over 48 inches

    @Override
    public void init() {

        
        // Build base follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startX, 0, 0));

        forwardsPath = new Path(new BezierLine(new Pose(startX, 0, 0), new Pose(endX, 0, 0)));
        forwardsPath.setConstantHeadingInterpolation(0);

        backwardsPath = new Path(new BezierLine(new Pose(endX, 0, 0), new Pose(startX, 0, 0)));
        backwardsPath.setConstantHeadingInterpolation(0);

        telemetry.addLine("Auto Tuner Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Automatically start the FF tuning phase
        state = State.TUNE_FF;
        lastTime = System.currentTimeMillis();
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        // Must update Pedro follower every loop to get current pose and errors
        follower.update();

        switch (state) {
            case TUNE_FF:
                // Slowly step up the raw motor power until the robot starts moving
                if (System.currentTimeMillis() - lastTime > 100) {
                    testPower += 0.01;
                    follower.setTeleOpDrive(testPower, 0, 0, true);
                    lastTime = System.currentTimeMillis();
                }

                // Check velocity magnitude. Constructing a Vector pointing Forward (0 radians).
                double currentVelocity = Math.abs(follower.getVelocity().dot(new Vector(1.0, 0)));
                
                // If moving faster than ~1 inch/second, we've overcome static friction!
                if (currentVelocity > 1.0) {
                    bestFF = testPower;
                    follower.setTeleOpDrive(0, 0, 0, true);
                    
                    // Pedro calculates friction mathematically using deceleration.
                    // We can estimate the rough equivalent for the constant.
                    Constants.followerConstants.setForwardZeroPowerAcceleration(-bestFF * 50); 
                    
                    // Transition to PID tuning
                    state = State.TUNE_P;
                    movingForward = true;
                    // Rebuild with baseline zero Constants
                    rebuildFollower();
                    follower.setStartingPose(new Pose(startX, 0, 0));
                    follower.followPath(forwardsPath);
                }
                break;

            case TUNE_P:
                tuneLoop(true);
                break;

            case TUNE_D:
                tuneLoop(false);
                break;

            case DONE:
                follower.setTeleOpDrive(0, 0, 0, true);
                telemetry.addLine("=== TUNING COMPLETE ===");
                telemetry.addData("Best P", bestP);
                telemetry.addData("Best D", bestD);
                telemetry.addData("Calculated Friction / FF Power", bestFF);
                telemetry.addLine("Update Constants.java with these values!");
                break;
        }

        // Live Debugging Telemetry
        telemetry.addData("State", state);
        if (state == State.TUNE_FF) {
            telemetry.addData("Test Power", testPower);
            telemetry.addData("X Velocity", follower.getVelocity().dot(new Vector(1.0, 0)));
        } else {
            telemetry.addData("Testing P", testP);
            telemetry.addData("Testing D", testD);
            telemetry.addData("Best Known Min Error", minError);
            telemetry.addData("Current Y Error", Math.abs(follower.getPose().getY()));
        }
        telemetry.update();
    }

    // Small helper to flush the mutated follower constants into a fresh Pedro Follower instance
    private void rebuildFollower() {
        // Modify the Translational PIDF in Constants manually
        // Keeping default integral and feedforward, overriding purely P and D
        Constants.followerConstants.translationalPIDFCoefficients(new PIDFCoefficients(testP, 0, testD, 0.025));
        follower = Constants.createFollower(hardwareMap);
    }

    // Handles sweeping back and forth and scoring tracking error
    private void tuneLoop(boolean tuningP) {
        // Collect cross-track error securely
        // Because target is Y=0, any Y deviation is pure cross-track error
        double yError = Math.abs(follower.getPose().getY());
        double headingError = Math.abs(follower.getPose().getHeading());
        
        // Weigh heading error heavily (radians vs inches)
        errorAccumulator += (yError + headingError * 15.0);
        errorSamples++;

        // End of the current sub-path
        if (!follower.isBusy()) {
            if (movingForward) {
                // At EndX, command to turn around
                movingForward = false;
                follower.followPath(backwardsPath);
            } else {
                // Back at StartX, one full cycle (forward+backwards) is complete
                movingForward = true;
                double avgError = errorAccumulator / Math.max(1, errorSamples);
                cycleErrors.add(avgError);

                // Check if our new parameter improved control
                if (avgError < minError) {
                    minError = avgError;
                    if (tuningP) {
                        bestP = testP;
                        testP += 0.05; // Increment P by a small step
                    } else {
                        bestD = testD;
                        testD += 0.01; // Increment D by a small step
                    }
                } else {
                    // Performance degraded, likely oscillating or overshooting
                    if (tuningP) {
                        state = State.TUNE_D; // Move on to D 
                        testP = bestP;        // Revert to best stable P
                        minError = Double.MAX_VALUE; // Reset score baseline
                    } else {
                        state = State.DONE; // Finished tuning entirely
                        testD = bestD;      // Revert to best stable D
                    }
                }

                errorAccumulator = 0.0;
                errorSamples = 0;

                // Restart the cycle if still tuning
                if (state != State.DONE) {
                    rebuildFollower();
                    follower.setStartingPose(new Pose(startX, 0, 0));
                    follower.followPath(forwardsPath);
                }
            }
        }
    }
}
