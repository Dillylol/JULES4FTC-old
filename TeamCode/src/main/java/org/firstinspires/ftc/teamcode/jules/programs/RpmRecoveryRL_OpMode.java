package org.firstinspires.ftc.teamcode.jules.programs; // Using jules.programs package for organization

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.shooter.JULES_ShooterController;
import org.firstinspires.ftc.teamcode.jules.JulesRamTx;
import org.firstinspires.ftc.teamcode.jules.JulesService;
import org.firstinspires.ftc.teamcode.jules.Metrics;

import java.util.Locale;

/**
 * RpmRecoveryRL_OpMode
 * Supports the 'RpmRecoveryRL' JULES program.
 * 
 * Features:
 * - Uses JULES_ShooterController.
 * - Test Mode (Y): Runs 3 consecutive Ramp-Up -> Target -> Off cycles.
 * - Recovery Mode (Manual):
 *   - Gamepad A: Intake Pulse.
 *   - Gamepad RT: Shooter + Boot (Servos).
 *   - LED Logic remaining active.
 */
@TeleOp(name = "RpmRecoveryRL Host", group = "JULES")
public class RpmRecoveryRL_OpMode extends OpMode {

    private BjornHardware hardware;
    private JULES_ShooterController shooter;
    private JulesRamTx julesTx;
    
    private boolean testRunning = false;
    private int testCycle = 0;
    private long stateTimerMs = 0;
    private TestState testState = TestState.IDLE;
    
    private enum TestState {
        IDLE,
        RAMPING_UP,
        HOLDING,
        SPIN_DOWN,
        WAIT_FOR_ZERO
    }
    
    // Constants for Test
    private static final int TEST_RPM = 2800; // Target
    private static final long HOLD_TIME_MS = 2000;
    private static final int TOTAL_CYCLES = 3;

    // Inputs
    private boolean yPrev = false;
    private boolean aPrev = false;

    @Override
    public void init() {
        hardware = BjornHardware.forTeleOp(hardwareMap);
        shooter = new JULES_ShooterController(hardware);
        
        // Initialize JULES stream
        julesTx = JulesService.newTransmitter(null, telemetry, "rpm_recovery_rl");
        
        telemetry.addLine("RpmRecoveryRL Host Initialized");
        telemetry.addLine("Press 'Y' to start 3-cycle Ramp Test.");
        telemetry.addLine("Use 'A' for Intake, 'RT' for Manual Recovery/Shoot.");
    }
    
    @Override
    public void start() {
        // Ensure subsystems are off
        shooter.setTargetRpm(0);
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        
        // --- Inputs ---
        if (gamepad1.y && !yPrev) {
            if (!testRunning) {
                startTest(nowMs);
            } else {
                stopTest(); // Cancel
            }
        }
        yPrev = gamepad1.y;
        
        boolean pulseIntake = gamepad1.a;
        boolean shootManual = gamepad1.right_trigger > 0.5;
        
        // --- Test Architecture ---
        if (testRunning) {
            runTestLogic(nowMs);
        } else {
            // Manual Mode / Recovery Test
            shooter.setTargetRpm(shootManual ? TEST_RPM : 0);
            
            // Intake Control
            if (hardware.intake != null) {
                hardware.intake.setPower(pulseIntake ? 1.0 : 0.0);
            }
            
            // Servo Control (Shoot) linked to RT
            if (shootManual) {
                 if (hardware.grip1 != null) hardware.grip1.setPower(1.0);
                 if (hardware.grip2 != null) hardware.grip2.setPower(1.0);
            } else {
                 if (hardware.grip1 != null) hardware.grip1.setPower(0.0);
                 if (hardware.grip2 != null) hardware.grip2.setPower(0.0);
            }
        }
        
        // --- Update Controller ---
        shooter.update();
        
        // --- Telemetry / Stream ---
        logMetrics(nowMs);
    }
    
    private void startTest(long nowMs) {
        testRunning = true;
        testCycle = 0;
        testState = TestState.RAMPING_UP;
        shooter.setTargetRpm(TEST_RPM);
        stateTimerMs = nowMs;
    }
    
    private void stopTest() {
        testRunning = false;
        testState = TestState.IDLE;
        shooter.setTargetRpm(0);
    }
    
    private void runTestLogic(long nowMs) {
        switch (testState) {
            case RAMPING_UP:
                if (shooter.isReady()) {
                    testState = TestState.HOLDING;
                    stateTimerMs = nowMs;
                }
                break;
                
            case HOLDING:
                if (nowMs - stateTimerMs > HOLD_TIME_MS) {
                    shooter.setTargetRpm(0);
                    testState = TestState.SPIN_DOWN;
                }
                break;
                
            case SPIN_DOWN:
                if (shooter.getMeasuredRpm() < 50) { // Approx 0
                     testState = TestState.WAIT_FOR_ZERO;
                     stateTimerMs = nowMs; // Wait a bit at 0?
                }
                break;
                
            case WAIT_FOR_ZERO:
                // Small delay at 0 before restart
                if (nowMs - stateTimerMs > 1000) {
                    testCycle++;
                    if (testCycle < TOTAL_CYCLES) {
                        // RESTART
                        shooter.setTargetRpm(TEST_RPM);
                        testState = TestState.RAMPING_UP;
                        stateTimerMs = nowMs;
                    } else {
                        // DONE
                        stopTest();
                    }
                }
                break;
                
            default:
                stopTest();
                break;
        }
    }
    
    private void logMetrics(long nowMs) {
        Metrics m = new Metrics();
        m.t = nowMs / 1000.0;
        m.batteryV = hardware.getBatteryVoltage();
        
        // Custom JSON for JULES analysis
        m.jsonData = String.format(Locale.US,
            "{\"rpm\":%.1f, \"target\":%d, \"ready\":%b, \"cycle\":%d, \"state\":\"%s\"}",
            shooter.getMeasuredRpm(),
            shooter.getTargetRpm(),
            shooter.isReady(),
            testCycle,
            testState.toString()
        );
        
        julesTx.send(m);
        
        telemetry.addData("State", testRunning ? testState.toString() : "MANUAL");
        telemetry.addData("RPM", "%.0f / %d", shooter.getMeasuredRpm(), shooter.getTargetRpm());
        telemetry.addData("Ready", shooter.isReady());
        telemetry.addData("Cycle", "%d / %d", testCycle, TOTAL_CYCLES);
    }
    
    @Override
    public void stop() {
        if (julesTx != null) julesTx.close();
    }
}
