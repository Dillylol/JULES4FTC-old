package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

import java.util.List;

/**
 * Bjorn CV Debugger
 * 
 * Objective: Visualize AprilTag coordinates to verify X/Y/Z assignments and bearing logic.
 * Usage:
 * 1. Run OpMode.
 * 2. Place tag in front of robot.
 * 3. Observe X, Y, Z and Bearing.
 * 
 * Instructions:
 * - If you move Robot LEFT, Tag should move RIGHT (Positive X?). Check Telemetry.
 * - If you move Robot RIGHT, Tag should move LEFT (Negative X?). Check Telemetry.
 */
@TeleOp(name = "Bjorn CV Debugger", group = "Tests")
public class BjornCVDebugger extends OpMode {

    private AprilTagCamera aprilTagCamera;
    private IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, BjornConstants.Sensors.IMU);
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);
        imu.resetYaw();

        // Standard MJPEG 640x480 Start
        aprilTagCamera = new AprilTagCamera();
        aprilTagCamera.start(hardwareMap, null);

        telemetry.addLine("Initialized.");
        telemetry.addLine("Move robot and check telemetry to verify coordinates.");
    }

    @Override
    public void loop() {
        // IMU Heading
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Robot Yaw", "%.1f°", robotYaw);
        
        telemetry.addLine("--- DETECTIONS ---");

        List<TagObservation> detections = aprilTagCamera.pollDetections();
        
        if (detections.isEmpty()) {
            telemetry.addLine("No Tags Detected");
        } else {
            for (TagObservation obs : detections) {
                // Calculate Data
                // Z = Depth, X = Horizontal (Right+?), Y = Vertical (Down+?)
                double range = Math.hypot(obs.x, obs.z);
                double bearing = Math.toDegrees(Math.atan2(obs.x, obs.z));
                
                telemetry.addData("ID " + obs.id, "Range: %.2fm | Brg: %.1f°", range, bearing);
                telemetry.addData(" -> Raw", "X: %.3f, Y: %.3f, Z: %.3f", obs.x, obs.y, obs.z);
                
                // Diagnostic Helper
                String hint = "CENTER";
                if (obs.x > 0.05) hint = "Tag is RIGHT -> Turn RIGHT (Decrease Angle?)";
                else if (obs.x < -0.05) hint = "Tag is LEFT -> Turn LEFT (Increase Angle?)";
                
                telemetry.addData(" -> Hint", hint);
                telemetry.addLine("");
            }
        }
        
        telemetry.update();
    }

    @Override
    public void stop() {
        if (aprilTagCamera != null) {
            aprilTagCamera.close();
        }
    }
}
