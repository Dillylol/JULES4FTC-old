package org.firstinspires.ftc.teamcode.jules.tests;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;
import org.firstinspires.ftc.teamcode.jules.core.JulesRobot;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

@Autonomous(name = "JULES: Client Auto", group = "JULES")
public class JulesClientAuto extends OpMode {

    private JulesRobot robot;
    private final Queue<String> cmdQueue = new ConcurrentLinkedQueue<>();
    private JulesStreamBus.Subscription sub;
    private Thread listener;
    
    private boolean isMoving = false;

    @Override
    public void init() {
        robot = new JulesRobot(hardwareMap, telemetry);
        robot.init();
        
        if (robot.streamBus != null) {
            try {
                sub = robot.streamBus.subscribe();
                listener = new Thread(() -> {
                    while (!Thread.currentThread().isInterrupted()) {
                        try {
                            String line = sub.take();
                            if (line == null) break;
                            cmdQueue.offer(line);
                        } catch (Exception e) { break; }
                    }
                });
                listener.start();
            } catch (Exception e) {
                telemetry.addData("Bus Error", e.getMessage());
            }
        }
        telemetry.addLine("Waiting for Client Waypoints...");
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.update();
        processCommands();
        
        if (robot.follower != null) {
            if (isMoving && !robot.follower.isBusy()) {
                isMoving = false;
                robot.publish("{\"type\":\"event\",\"payload\":\"arrived\"}");
            }
            
            Pose p = robot.follower.getPose();
            telemetry.addData("Pose", String.format("%.1f, %.1f, %.1f", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        }
        telemetry.addData("Status", isMoving ? "Moving" : "Idle");
        telemetry.update();
    }

    private void processCommands() {
        String raw;
        while ((raw = cmdQueue.poll()) != null) {
            try {
                JsonElement el = GsonCompat.parse(raw);
                if (el.isJsonObject()) {
                    JsonObject obj = el.getAsJsonObject();
                    
                    // Unwrap if inside 'text' (Standard Bridge) or 'cmd' wrapper
                    if (obj.has("text")) {
                         String inner = obj.get("text").getAsString();
                         // Try parsing inner if it looks like JSON
                         if (inner.startsWith("{")) {
                             JsonElement innerEl = GsonCompat.parse(inner);
                             if (innerEl.isJsonObject()) obj = innerEl.getAsJsonObject();
                         }
                    }
                    
                    if (obj.has("cmd")) {
                        String cmd = obj.get("cmd").getAsString();
                        
                        if (cmd.equals("goto")) {
                            // Simple GOTO logic (Keep existing)
                            if (robot.follower == null) continue;
                            double x = obj.get("x").getAsDouble();
                            double y = obj.get("y").getAsDouble();
                            double h = obj.has("h") ? Math.toRadians(obj.get("h").getAsDouble()) : 0;
                            robot.follower.followPath(
                                robot.follower.pathBuilder()
                                    .addPath(new BezierLine(new Pose(robot.follower.getPose().getX(), robot.follower.getPose().getY()), new Pose(x, y)))
                                    .setLinearHeadingInterpolation(robot.follower.getPose().getHeading(), h)
                                    .build(),
                                true
                            );
                            isMoving = true;
                        }
                        
                        else if (cmd.equals("execute_pp")) {
                            if (robot.follower == null) {
                                telemetry.addLine("Error: No Follower");
                                continue;
                            }
                            if (obj.has("path")) {
                                JsonObject pathData = obj.getAsJsonObject("path");
                                buildAndRunPedroPath(pathData);
                            }
                        }
                    }
                }
            } catch (Exception e) {
                telemetry.addData("Cmd Error", e.getMessage());
            }
        }
    }

    private void buildAndRunPedroPath(JsonObject data) {
        try {
            if (!data.has("lines")) return;
            
            // Get Start Point
            JsonObject startObj = data.getAsJsonObject("startPoint");
            double startX = startObj.get("x").getAsDouble();
            double startY = startObj.get("y").getAsDouble();
            // Assuming startDeg converted to radians if needed, or Pedro uses radians? 
            // Pedro Pose uses Radians usually. .pp uses Degrees.
            double startH = Math.toRadians(startObj.get("startDeg").getAsDouble());
            
            // We usually assume the robot is already at start or we reset pose?
            // For safety, let's reset pose to start of path if we are running a full auto.
            // Or just assume current pose is close enough?
            // Let's reset for now to ensure path validity.
            robot.follower.setPose(new Pose(startX, startY, startH));
            
            PathBuilder builder = robot.follower.pathBuilder();
            
            // Iterate Lines
            for (JsonElement lineEl : data.getAsJsonArray("lines")) {
                JsonObject line = lineEl.getAsJsonObject();
                JsonObject endObj = line.getAsJsonObject("endPose");
                double endX = endObj.get("x").getAsDouble();
                double endY = endObj.get("y").getAsDouble();
                double endH = Math.toRadians(endObj.get("endDeg").getAsDouble());
                
                // Control Poses
                // If controlPoses defines curve.
                // .pp lines have "controlPoses": [{"x":..., "y":...}]
                // Beziers: 
                // 0 CP = Line
                // 1 CP = Quad/Curve (Pose, Control, Pose) -> BezierCurve(start, control, end)
                // 2 CP = Cubic (Pose, C1, C2, End) -> BezierCurve(start, c1, c2, end)
                
                Pose endPose = new Pose(endX, endY);
                
                // Getting previous point? Builder handles chaining usually.
                // But we need to construct the Bezier object.
                // Builder needs .addPath(BezierCurve/Line)
                // Bezier constructor needs Start Pose explicitly.
                // For the first line, Start is Path Start.
                // For subsequent, Start is Previous End.
                // Wait, PathBuilder keeps track? Yes.
                // But `new BezierLine(start, end)` requires start.
                
                // We need to track 'current' tail.
                
                // Let's rely on builder if possible, or manually track.
                // Since this loop rebuilds, we need a 'current tail'.
                // Initial tail is StartPose.
                
                // However, parsing FULL structure is complex.
                // Let's assume standard chain.
            }
            
            // SIMPLIFIED: Just parse Waypoints and use default pathing for now
            // To properly parse .pp we need a robust loop.
            // Let's do a simplified version: GOTO each endpoint.
            // This sacrifices the curves but gets the Super Controller working.
            // Converting Bezier JSON to Java Objects manually in one go is error prone without a library.
            
            // REVISED STRATEGY: 
            // 1. Get all endpoints.
            // 2. Chain them.
            
            PathChain chain = null;
            
            // Actually, let's try to do it right for at least Lines and Quad Curves.
            // We need a `lastPose`.
            Pose lastPose = new Pose(startX, startY);
            
            for (JsonElement lineEl : data.getAsJsonArray("lines")) {
                JsonObject line = lineEl.getAsJsonObject();
                JsonObject endObj = line.getAsJsonObject("endPose");
                Pose endP = new Pose(endObj.get("x").getAsDouble(), endObj.get("y").getAsDouble());
                double endHeading = Math.toRadians(endObj.get("endDeg").getAsDouble());

                // Check Control Poses
                if (line.has("controlPoses") && line.getAsJsonArray("controlPoses").size() > 0) {
                    // Curve
                    // We only support 1 or 2 control points for now
                    if (line.getAsJsonArray("controlPoses").size() == 1) {
                         JsonObject cpObj = line.getAsJsonArray("controlPoses").get(0).getAsJsonObject();
                         Pose cp = new Pose(cpObj.get("x").getAsDouble(), cpObj.get("y").getAsDouble());
                         builder.addPath(new BezierCurve(lastPose, cp, endP));
                    } else {
                         // Cubic?
                         JsonObject cp1Obj = line.getAsJsonArray("controlPoses").get(0).getAsJsonObject();
                         JsonObject cp2Obj = line.getAsJsonArray("controlPoses").get(1).getAsJsonObject();
                         Pose cp1 = new Pose(cp1Obj.get("x").getAsDouble(), cp1Obj.get("y").getAsDouble());
                         Pose cp2 = new Pose(cp2Obj.get("x").getAsDouble(), cp2Obj.get("y").getAsDouble());
                         builder.addPath(new BezierCurve(lastPose, cp1, cp2, endP));
                    }
                } else {
                    // Line
                    builder.addPath(new BezierLine(lastPose, endP));
                }
                
                builder.setLinearHeadingInterpolation(robot.follower.getPose().getHeading(), endHeading); // Simplified heading
                lastPose = endP;
            }
            
            // Execution
            robot.follower.followPath(builder.build(), true);
            isMoving = true;
            telemetry.addLine("Executing Loaded Path!");
            
        } catch (Exception e) {
            telemetry.addData("Path Build Error", e.toString());
        }
    }

    @Override
    public void stop() {
        if (listener != null) listener.interrupt();
        if (sub != null) {
            try { sub.close(); } catch (Exception e) {}
        }
    }
}
