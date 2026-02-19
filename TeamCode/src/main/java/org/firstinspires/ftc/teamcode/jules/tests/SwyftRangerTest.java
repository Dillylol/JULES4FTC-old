package org.firstinspires.ftc.teamcode.jules.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.SwyftRangerConstants;

@TeleOp(name = "Swyft Ranger Test", group = "Test")
public class SwyftRangerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize hardware using the shared wrapper
        // Note: This expects the full robot configuration to be present.
        BjornHardware robot = BjornHardware.forTeleOp(hardwareMap);

        telemetry.addLine("Swyft Ranger Test Initialized");
        telemetry.addLine("Sensor: " + SwyftRangerConstants.HARDWARE_NAME);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double voltage = robot.swyftRanger.getVoltage();
            
            // Calculate distances for different modes
            double battVolts = 13.0;
            try {
                battVolts = hardwareMap.voltageSensor.iterator().next().getVoltage();
            } catch (Exception ignored) {}

            // Calculate distances
            double distInches = SwyftRangerConstants.voltageToInches(voltage, battVolts);
            
            telemetry.addData("Raw Voltage", "%.3f V", voltage);
            telemetry.addData("Batt Voltage", "%.2f V", battVolts);
            telemetry.addData("Distance", "%.2f in", distInches);
            telemetry.addData("Distance", "%.2f cm", distInches * 2.54);
            
            // Helpful visual logic:
            if (voltage < SwyftRangerConstants.MIN_VOLTAGE) {
                telemetry.addLine("STATUS: Too Close / No Signal");
            } else {
                 telemetry.addLine("STATUS: Valid");
            }

            telemetry.update();
        }
    }
}
