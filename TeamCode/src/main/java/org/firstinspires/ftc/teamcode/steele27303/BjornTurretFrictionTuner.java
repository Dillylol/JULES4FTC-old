package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.common.BjornHardware;

/**
 * Bjorn Turret Friction Tuner
 * 
 * Purpose: Find the minimum power (kS) required to overcome static friction.
 * 
 * Controls:
 * - D-Pad Up: Increase Power (+0.01)
 * - D-Pad Down: Decrease Power (-0.01)
 * - L-Bumper + D-Pad: Larger Intervals (+/- 0.05)
 * - A: Hold to apply Power (Rotates towards positive limits)
 * - B: Stop / Reset
 */
@TeleOp(name = "Bjorn Turret Friction Tuner", group = "Test")
public class BjornTurretFrictionTuner extends BjornTeleBase {

    private DcMotorEx turret;
    private double testPower = 0.0;
    
    // Limits
    private static final double HARD_MAX = 170.0;
    private static final double HARD_MIN = -10.0;

    // State
    private int lastEncoderPos = 0;
    private double turretAngleDeg = 0;
    
    // Inputs
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    @Override
    public void init() {
        initSubsystems();
        turret = hardware.turret;
        
        // Override Direction (User fix)
        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        telemetry.addLine("=== FRICTION TUNER ===");
        telemetry.addLine("Find Min Power (kS)");
        telemetry.update();
    }

    @Override
    protected String getAllianceName() { return "TEST"; }

    @Override
    protected int getGoalTagId() { return 0; }

    @Override
    public void loop() {
        // --- Input: Adjust Power ---
        boolean largeStep = gamepad1.left_bumper;
        double step = largeStep ? 0.05 : 0.01;
        
        if (gamepad1.dpad_up && !dpadUpPrev) {
            testPower += step;
        }
        dpadUpPrev = gamepad1.dpad_up;
        
        if (gamepad1.dpad_down && !dpadDownPrev) {
            testPower -= step;
        }
        dpadDownPrev = gamepad1.dpad_down;
        
        // Clamp
        if (testPower < 0) testPower = 0;
        if (testPower > 1.0) testPower = 1.0;
        
        // --- READ STATE ---
        int currentPos = turret.getCurrentPosition();
        int deltaTicks = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;
        turretAngleDeg += (deltaTicks / BjornHardware.TURRET_TICKS_PER_DEGREE);
        
        // --- RUN LOGIC ---
        // A Button: Run Open Loop
        boolean run = gamepad1.a;
        if (run) {
            // Safety
            if (turretAngleDeg > HARD_MAX) {
                turret.setPower(0);
                telemetry.addData("STATUS", "LIMIT HIT");
            } else {
                turret.setPower(testPower);
                telemetry.addData("STATUS", "APPLYING POWER");
            }
        } else {
            turret.setPower(0);
            telemetry.addData("STATUS", "IDLE");
        }
        
        if (gamepad1.b) {
            turret.setPower(0);
            // Optional: Reset encoder?
        }
        
        telemetry.addData("Test Power", "%.2f", testPower);
        telemetry.addData("Angle", "%.1f", turretAngleDeg);
        telemetry.addData("Velocity Ticks", deltaTicks);
        telemetry.update();
    }
}
