package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Dead Wheel Diagnostic", group = "Diagnostic")
public class DeadWheelDiagnostic extends LinearOpMode {

    private DcMotorEx leftEncoder;
    private DcMotorEx rightEncoder;
    private DcMotorEx strafeEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        // Encoders are typically plugged into motor ports
        leftEncoder = hardwareMap.get(DcMotorEx.class, "lf");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "lr");
        strafeEncoder = hardwareMap.get(DcMotorEx.class, "rf");

        // Reset encoders
        leftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        strafeEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Dead Wheel Diagnostic Initialized");
        telemetry.addLine("Push the robot and check telemetry for changes.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Apply directions from Constants/Robot configuration
            // Left: REVERSE (-1), Right: REVERSE (-1), Strafe: FORWARD (1)
            int leftPos = leftEncoder.getCurrentPosition() * -1;
            int rightPos = rightEncoder.getCurrentPosition() * -1;
            int strafePos = strafeEncoder.getCurrentPosition() * 1;

            telemetry.addData("Left Encoder (lf) [REVERSED]", leftPos);
            telemetry.addData("Right Encoder (lr) [REVERSED]", rightPos);
            telemetry.addData("Strafe Encoder (rf) [FORWARD]", strafePos);
            telemetry.addLine("\n--- Expected Directions ---");
            telemetry.addLine("Pushing FORWARD: Left & Right should INCREASE");
            telemetry.addLine("Pushing RIGHT: Strafe should INCREASE");
            telemetry.update();
        }
    }
}
