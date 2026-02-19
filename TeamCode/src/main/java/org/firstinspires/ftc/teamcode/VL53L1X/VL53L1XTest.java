package org.firstinspires.ftc.teamcode.VL53L1X;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "VL53L1X Test", group = "Sensor")
public class VL53L1XTest extends OpMode {

    private VL53L1X sensor;

    @Override
    public void init() {
        try {
            sensor = hardwareMap.get(VL53L1X.class, "vl53l1x");

            // 1. Force a "clean" bus reset logic (optional but helpful)
            // (No direct API for this, but lowering speed helps)

            // 2. Initialize
            sensor.initialize();

            telemetry.addData("Status", "Found VL53L1X Driver instance");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find 'vl53l1x' configuration.");
            telemetry.addData("Detail", e.getMessage());
        }
    }

    @Override
    public void loop() {
        if (sensor != null) {
            boolean connected = sensor.isConnected();
            int modelId = 0;
            if (connected || true) {
                modelId = sensor.getModelID();
            }

            telemetry.addData("Connection", connected ? "ESTABLISHED" : "FAILED (Check ID)");
            telemetry.addData("Model ID (Hex)", String.format("0x%04X", modelId));
            telemetry.addData("Expected ID", "0xEACC (approx)");
        } else {
            telemetry.addData("Status", "Waiting for init...");
        }
        telemetry.update();
    }
}
