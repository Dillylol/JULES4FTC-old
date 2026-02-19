package org.firstinspires.ftc.teamcode.steele27303;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.shooter.JULES_ShooterController;
import org.firstinspires.ftc.teamcode.jules.JulesRamTx;
import org.firstinspires.ftc.teamcode.jules.JulesService;
import org.firstinspires.ftc.teamcode.jules.Metrics;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;

import java.util.Locale;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Special OpMode for tuning flywheel ramp-up and recovery.
 * <p>
 * Controls:
 * - A (Hold): Intake On (Feed)
 * - RT (Hold): Grip Servos Spin
 * - B (Toggle): Set Target RPM (2800)
 * - DPad Up/Down: Manuel RPM Tune (+/- 50)
 * - Y / START: Trigger 3-Cycle Auto Test
 * <p>
 * Data is streamed to JULES via JulesRamTx ("rpm_recovery_rl").
 */
@TeleOp(name = "Flywheel Ramp Tuner", group = "Tuning")
public class FlywheelRampTuner extends BjornTeleBase {

    private JulesRamTx julesTx;
    private JulesStreamBus streamBus;
    private JulesStreamBus.Subscription busSub;
    private Thread busPump;
    private final Queue<String> pendingCmds = new ConcurrentLinkedQueue<>();
    
    private JULES_ShooterController julesShooter;
    
    private long startTime;
    // State
    private int customTargetRpm = 2800;
    private int cycleCount = 0;
    
    // State

    // Removed internal TestState state machine
    
    // De-bounce input
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private boolean dpadLeftPrev = false;
    private boolean yPrev = false;
    private boolean startPrev = false;
    private boolean bPrev = false;

    @Override
    public void init() {
        super.initSubsystems();
        
        // Use JULES Shooter Controller
        julesShooter = new JULES_ShooterController(hardware);
        
        // Initialize Rx (and prepare Bridge)
        setupJulesRx();
        
        // Initialize JULES transmitter (must happen AFTER bridge is prepared)
        julesTx = JulesService.newTransmitter(null, telemetry, "rpm_recovery_rl");
        
        telemetry.addLine("Flywheel Ramp Tuner Ready");
        telemetry.addLine("JULES: Full Control");
        telemetry.addLine("B: Toggle Manual RPM (Local Override)");
        telemetry.update();
    }
    
    private void setupJulesRx() {
        JulesBridgeManager mgr = JulesBridgeManager.getInstance();
        if (mgr != null) {
            mgr.prepare(hardwareMap.appContext);
            streamBus = mgr.getStreamBus();
            if (streamBus != null) {
                try {
                    busSub = streamBus.subscribe();
                    busPump = new Thread(() -> {
                        try {
                            for (;;) {
                                String line = busSub.take();
                                if (line == null) break;
                                pendingCmds.offer(line);
                            }
                        } catch (InterruptedException ignored) { }
                    }, "JULES-TunerPump");
                    busPump.setDaemon(true);
                    busPump.start();
                } catch (Exception ignored) {}
            }
        }
    }

    @Override
    public void start() {
        startTime = System.nanoTime();
        super.start();
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        double nowSec = (System.nanoTime() - startTime) / 1e9;
        
        // --- Rx Handling ---
        handleIncomingCommands();

        // --- Input (Manual Overrides) ---
        if (gamepad1.dpad_up && !dpadUpPrev) customTargetRpm += 50;
        dpadUpPrev = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownPrev) customTargetRpm -= 50;
        dpadDownPrev = gamepad1.dpad_down;
        
        // Toggle Enable/Disable locally
        if (gamepad1.b && !bPrev) shooterActive = !shooterActive;
        bPrev = gamepad1.b;

        // --- Subsystem ---
        handleManualSystems();
        
        // --- Control Logic ---
        // Just apply the target. JULES controls 'customTargetRpm' via commands.
        // Local B button acts as a "Kill Switch" / "Enable Switch"
        if (shooterActive) {
            julesShooter.setTargetRpm(customTargetRpm);
        } else {
            julesShooter.setTargetRpm(0);
        }
        
        // Update Controller
        julesShooter.update();
        
        // --- Metrics ---
        Metrics m = new Metrics();
        m.t = nowSec;
        double batt = getBatteryVoltage();
        m.batteryV = batt;
        m.headingDeg = 0; 
        
        // JSON Packet
        // State: IDLE, RAMP, or READY
        String stateStr = "IDLE";
        if (julesShooter.getTargetRpm() > 0) {
            stateStr = julesShooter.isReady() ? "READY" : "RAMP";
        }
        
        m.jsonData = String.format(Locale.US, 
            "{\"target\":%d, \"rpm\":%.2f, \"ready\":%b, \"volts\":%.2f, \"cycle\":%d, \"state\":\"%s\", \"mode\":\"EXTERNAL\"}",
            julesShooter.getTargetRpm(),
            julesShooter.getMeasuredRpm(),
            julesShooter.isReady(),
            batt,
            cycleCount,
            stateStr
        );
        
        julesTx.send(m);
        
        // Telemetry
        telemetry.addData("System", isArmed ? "ARMED - LISTENING" : "DISARMED (Press Y)");
        telemetry.addData("Ctrl", shooterActive ? "ACTIVE" : "IDLE");
        telemetry.addData("Target", julesShooter.getTargetRpm());
        telemetry.addData("RPM", "%.0f", julesShooter.getMeasuredRpm());
        telemetry.addData("Ready", julesShooter.isReady());
    }
    
    // Removed internal AutoTest logic
    
    // Puppet Mode State
    private boolean isArmed = false;

    private void handleIncomingCommands() {
        while (!pendingCmds.isEmpty()) {
            String raw = pendingCmds.poll();
            if (raw == null) continue;

            try {
                // recursively unwrap if it's a Bridge message {"type":"cmd", "text": "..."}
                String cmd = raw.trim();
                JsonObject root = null;
                
                if (cmd.startsWith("{") && cmd.endsWith("}")) {
                    JsonElement el = GsonCompat.parse(cmd);
                    if (el != null && el.isJsonObject()) {
                        root = el.getAsJsonObject();
                        if (root.has("type") && root.has("text")) {
                            // Bridge wrapper -> unwrap
                             JsonElement txt = root.get("text");
                             if (txt.isJsonObject()) {
                                 root = txt.getAsJsonObject();
                             } else if (txt.isJsonPrimitive()) {
                                 String inner = txt.getAsString();
                                 if (inner.startsWith("{")) {
                                     JsonElement innerEl = GsonCompat.parse(inner);
                                     if (innerEl.isJsonObject()) root = innerEl.getAsJsonObject();
                                 } else {
                                     cmd = inner; // It's just a string command inside text
                                     root = null;
                                 }
                             }
                        }
                    }
                }
                
                // If we have a JSON object for the command payload
                if (root != null) {
                    if (root.has("cmd")) {
                         String c = root.get("cmd").getAsString();
                         String v = root.has("val") ? root.get("val").getAsString() : "";
                         cmd = c + " " + v;
                    }
                }

                // Normalization
                cmd = cmd.toLowerCase(Locale.US).trim();

                // --- Safety Check ---
                if (!isArmed) {
                    // Ignore all motion commands if not armed
                    continue;
                }

                if (cmd.startsWith("set_rpm")) {
                    String[] parts = cmd.split(" ");
                    if (parts.length > 1) {
                        try {
                            int newRpm = (int) Double.parseDouble(parts[1]);
                            // If we are starting a new run (going from 0/low to high), increment cycle
                            if (newRpm > 0 && customTargetRpm == 0) {
                                cycleCount++;
                            }
                            customTargetRpm = newRpm;
                            shooterActive = true; 
                        } catch (Exception ignored) {}
                    }
                } else if (cmd.startsWith("set_ramp")) {
                    String[] parts = cmd.split(" ");
                    if (parts.length > 1) {
                         try {
                            double sec = Double.parseDouble(parts[1]);
                            julesShooter.setRampDuration((long)(sec * 1000.0));
                        } catch (Exception ignored) {}
                    }
                } else if (cmd.startsWith("set_ff")) {
                    String[] parts = cmd.split(" ");
                    if (parts.length > 1) {
                        try {
                            double ff = Double.parseDouble(parts[1]);
                            julesShooter.setFeedforward(ff);
                        } catch (Exception ignored) {}
                    }
                }
            } catch (Exception ignored) {}
        }
    }

    private void handleManualSystems() {
        // Y -> Toggle Armed (Puppet Mode)
        if (gamepad1.y && !yPrev) {
            isArmed = !isArmed;
            // Safety: Disarming immediately kills motors
            if (!isArmed) {
                shooterActive = false;
                julesShooter.setTargetRpm(0);
            }
        }
        yPrev = gamepad1.y;

        // A -> Intake
        double intakePower = gamepad1.a ? 1.0 : (gamepad1.x ? -1.0 : 0.0);
        if (intake != null) intake.setPower(intakePower);
        
        // RT -> Grips
        boolean rt = gamepad1.right_trigger > 0.5;
        double gripPower = rt ? 1.0 : (gamepad1.left_trigger > 0.5 ? -1.0 : 0.0);
        if (grip1 != null) grip1.setPower(gripPower);
        if (grip2 != null) grip2.setPower(gripPower);
    }

    @Override
    public void stop() {
        if (julesTx != null) julesTx.close();
        if (busSub != null) try { busSub.close(); } catch (Exception ignored) {}
        if (busPump != null) busPump.interrupt();
        super.stop();
    }
    
    private double getBatteryVoltage() {
        // Try hardware map first, else fallback
        // if (julesShooter != null) return 12.0; // REMOVED: Caused hardcoded voltage
        double result = 12.0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) {
                result = v;
                break;
            }
        }
        return result;
    }

    @Override protected String getAllianceName() { return "Tuner"; }
    @Override protected int getGoalTagId() { return -1; }
}
