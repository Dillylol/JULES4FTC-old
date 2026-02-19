package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turret Test 20:1", group = "Test")
public class TurretTest20to1 extends LinearOpMode {

    private DcMotorEx turret;

    // --- Constants ---
    // Motor: Rev HD Hex (28 ticks/rev)
    // Gear Ratio: 20:1
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 20.0;
    private static final double TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;

    // PID Constants (75% reduction from original kP=0.015, kD=0.004)
    private static final double kP = 0.00375;
    private static final double kI = 0.0;
    private static final double kD = 0.001;
    private static final double kF = 0.0;

    private static final double POWER_CAP = 1.0;
    private static final double MANUAL_RATE_DEG_PER_SEC = 90.0; // Speed for manual target adjustment

    // --- State ---
    private double targetAngle = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;
    private int startPos = 0;

    @Override
    public void runOpMode() {
        // Init
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        
        // Reset Encoder
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // We handle control loop ourselves
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Ensure direction matches standard (Forward?)
        // In other files it was FORWARD.
        turret.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Turret Test 20:1 Initialized");
        telemetry.addData("Ratio", "%.1f", GEAR_RATIO);
        telemetry.update();

        waitForStart();

        lastTime = System.currentTimeMillis();
        startPos = turret.getCurrentPosition();

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (dt == 0) dt = 0.001; // Avoid divide by zero
            lastTime = now;

            // --- Input ---
            // GP1 Left Stick X controls Target Angle
            // Left (Negative) -> Decrease Target
            // Right (Positive) -> Increase Target
            double input = gamepad1.left_stick_x;
            
            if (Math.abs(input) > 0.05) {
                targetAngle += input * MANUAL_RATE_DEG_PER_SEC * dt;
            }
            
            // Limit Target Range? 
            // Previous limits were 0 to 180. Let's keep it safe.
            targetAngle = Range.clip(targetAngle, 0, 180);

            // --- Feedback ---
            int currentPos = turret.getCurrentPosition();
            double currentAngle = (currentPos - startPos) / TICKS_PER_DEGREE;

            // --- PID ---
            double error = targetAngle - currentAngle;
            double derivative = (error - lastError) / dt;
            
            double pOut = kP * error;
            double dOut = kD * derivative;
            
            double output = pOut + dOut + kF;
            
            // --- Output ---
            // Apply Power Cap
            output = Range.clip(output, -POWER_CAP, POWER_CAP);
            
            turret.setPower(output);
            
            lastError = error;

            // --- Telemetry ---
            telemetry.addData("Target", "%.1f", targetAngle);
            telemetry.addData("Actual", "%.1f", currentAngle);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("Power", "%.2f", output);
            telemetry.addData("Loop dt", "%.3f s", dt);
            telemetry.update();
        }
    }
}
