package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.jules.JulesRamTx;
import org.firstinspires.ftc.teamcode.jules.JulesService;
import org.firstinspires.ftc.teamcode.jules.Metrics;

import java.util.Locale;

/**
 * Generic Data Streamer for JULES.
 * Streams Battery Voltage and IMU data for analysis.
 */
@TeleOp(name = "JULES Data Streamer", group = "Analysis")
public class JulesDataStreamer extends LinearOpMode {

    private JulesRamTx julesTx;
    private BjornHardware robot;

    @Override
    public void runOpMode() {
        // Initialize Hardware
        robot = BjornHardware.forTeleOp(hardwareMap);
        
        // Initialize JULES Transmitter
        // Topic: "data_stream"
        julesTx = JulesService.newTransmitter(null, telemetry, "data_stream");

        telemetry.addLine("JULES Data Streamer Ready");
        telemetry.addLine("Topic: 'data_stream'");
        telemetry.update();

        waitForStart();

        // 50Hz Loop (approx)
        while (opModeIsActive()) {
            double startTime = System.nanoTime();
            
            // 1. Read Sensors
            double voltage = robot.getBatteryVoltage();
            YawPitchRollAngles angles = robot.imu.getRobotYawPitchRollAngles();
            
            double yaw = angles.getYaw(AngleUnit.DEGREES);
            double pitch = angles.getPitch(AngleUnit.DEGREES);
            double roll = angles.getRoll(AngleUnit.DEGREES);

            // 2. Build Metrics Packet
            Metrics m = new Metrics();
            m.t = System.currentTimeMillis() / 1000.0;
            m.batteryV = voltage;
            m.headingDeg = yaw;
            
            // Custom JSON payload
            m.jsonData = String.format(Locale.US,
                "{\"volts\":%.3f, \"yaw\":%.2f, \"pitch\":%.2f, \"roll\":%.2f}",
                voltage, yaw, pitch, roll
            );
            
            // 3. Send
            julesTx.send(m);

            // 4. Telemetry
            telemetry.addData("Volts", "%.2f V", voltage);
            telemetry.addData("Yaw", "%.1f", yaw);
            telemetry.addData("Pitch", "%.1f", pitch);
            telemetry.addData("Roll", "%.1f", roll);
            telemetry.update();
            
            // Limit loop rate to ~50Hz
            while (opModeIsActive() && (System.nanoTime() - startTime) < 20_000_000) {
                sleep(1);
            }
        }
        
        julesTx.close();
    }
}
