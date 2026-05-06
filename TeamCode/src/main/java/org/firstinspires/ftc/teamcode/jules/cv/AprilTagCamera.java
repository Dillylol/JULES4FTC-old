package org.firstinspires.ftc.teamcode.jules.cv;

import android.util.Size;
import androidx.annotation.Nullable;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Lightweight compile-safe AprilTag camera facade.
 *
 * This preserves the API used across OpModes even when CV backend code is absent.
 */
public final class AprilTagCamera implements AutoCloseable {
    private static final Size RESOLUTION = new Size(1280, 720);
    private static final double INCH_TO_METER = 0.0254;

    public static final class TagObservation {
        public final int id;
        public final String tagClass;
        public final double x;
        public final double y;
        public final double z;
        public final double yaw;
        public final double pitch;
        public final double roll;
        public final long timestampMs;

        public TagObservation(int id, String tagClass, double x, double y, double z,
                              double yaw, double pitch, double roll, long timestampMs) {
            this.id = id;
            this.tagClass = tagClass;
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.pitch = pitch;
            this.roll = roll;
            this.timestampMs = timestampMs;
        }
    }

    private final List<TagObservation> latestDetections = new ArrayList<>();
    @Nullable
    private TagObservation latestGoalObservation;
    @Nullable
    private JulesStreamBus streamBus;
    private boolean streaming;
    @Nullable
    private VisionPortal visionPortal;
    @Nullable
    private AprilTagProcessor aprilTag;

    public void start(HardwareMap hardwareMap, @Nullable JulesStreamBus streamBus) {
        start(hardwareMap, streamBus, 1.0);
    }

    public void start(HardwareMap hardwareMap, @Nullable JulesStreamBus streamBus, double decimation) {
        this.streamBus = streamBus;
        close();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(CameraConfig.FX, CameraConfig.FY, CameraConfig.CX, CameraConfig.CY)
                .build();
        aprilTag.setDecimation((float) decimation);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CameraConfig.WEBCAM_NAME))
                .setCameraResolution(RESOLUTION)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();
        streaming = true;
    }

    public List<TagObservation> pollDetections() {
        if (aprilTag == null) {
            latestDetections.clear();
            latestGoalObservation = null;
            return Collections.emptyList();
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        latestDetections.clear();

        if (detections != null) {
            for (AprilTagDetection d : detections) {
                if (d == null || d.ftcPose == null) {
                    continue;
                }
                TagObservation obs = new TagObservation(
                        d.id,
                        CameraConfig.classify(d.id),
                        d.ftcPose.x * INCH_TO_METER,
                        d.ftcPose.y * INCH_TO_METER,
                        d.ftcPose.z * INCH_TO_METER,
                        d.ftcPose.yaw,
                        d.ftcPose.pitch,
                        d.ftcPose.roll,
                        System.currentTimeMillis());
                latestDetections.add(obs);
            }
        }

        TagObservation goal = null;
        for (TagObservation obs : latestDetections) {
            if (obs.id == CameraConfig.BLUE_GOAL_TAG_ID || obs.id == CameraConfig.RED_GOAL_TAG_ID) {
                goal = obs;
                break;
            }
        }
        latestGoalObservation = goal;
        return new ArrayList<>(latestDetections);
    }

    public void publishDetections() {
        if (streamBus == null) {
            return;
        }
        List<TagObservation> detections = pollDetections();
        JsonObject root = new JsonObject();
        root.addProperty("type", "apriltag_detections");
        root.addProperty("ts", System.currentTimeMillis());
        JsonArray arr = new JsonArray();
        for (TagObservation obs : detections) {
            JsonObject item = new JsonObject();
            item.addProperty("id", obs.id);
            item.addProperty("tagClass", obs.tagClass);
            item.addProperty("x", obs.x);
            item.addProperty("y", obs.y);
            item.addProperty("z", obs.z);
            item.addProperty("yaw", obs.yaw);
            item.addProperty("pitch", obs.pitch);
            item.addProperty("roll", obs.roll);
            arr.add(item);
        }
        root.add("detections", arr);
        streamBus.publishJsonLine(root.toString());
    }

    public boolean isStreaming() {
        if (visionPortal == null) {
            return false;
        }
        return visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public int getLastDetectionCount() {
        return latestDetections.size();
    }

    public void configureCameraControls() {
        setManualExposure(CameraConfig.TUNED_EXPOSURE, CameraConfig.TUNED_GAIN);
    }

    public void setManualExposure(int exposureMs, int gain) {
        if (visionPortal == null) {
            return;
        }
        ExposureControl expCtrl = visionPortal.getCameraControl(ExposureControl.class);
        if (expCtrl != null) {
            if (expCtrl.getMode() != ExposureControl.Mode.Manual) {
                expCtrl.setMode(ExposureControl.Mode.Manual);
            }
            expCtrl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
        }
        GainControl gainCtrl = visionPortal.getCameraControl(GainControl.class);
        if (gainCtrl != null) {
            gainCtrl.setGain(gain);
        }
    }

    @Nullable
    public TagObservation getLatestGoalObservation() {
        return latestGoalObservation;
    }

    @Override
    public void close() {
        streaming = false;
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        aprilTag = null;
        latestDetections.clear();
        latestGoalObservation = null;
    }
}
