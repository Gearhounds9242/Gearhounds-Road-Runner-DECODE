package org.firstinspires.ftc.teamcode.Utilities;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * VisionSystem: manages webcam + AprilTag detection.
 * Provides clean data for:
 *   - forward distance (inches)
 *   - lateral offset (inches)
 *   - yaw angle (degrees)
 * Handles exposure and gain setup automatically.
 */
public class VisionSystem {

    /** Data packet passed to TeleOp for auto-alignment */
    public static class TagData {
        public boolean hasTag = false;
        public int id = -1;
        public double forwardIn = 0.0;   // Tag distance forward/back relative to camera (inches)
        public double lateralIn = 0.0;   // Tag offset left/right relative to camera (inches)
        public double yawDeg = 0.0;      // Tag yaw relative to camera (degrees)
    }

    private final VisionPortal visionPortal;
    private final AprilTagProcessor tagProcessor;

    /** Constructor: builds AprilTag pipeline and configures the webcam */
    public VisionSystem(HardwareMap hardwareMap) {

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Wait until streaming begins before setting exposure/gain
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        // Manual exposure
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl != null && exposureControl.isExposureSupported()) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS);
        }

        // Manual gain
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(80);
        }
    }

    /** Returns latest TagData (hasTag=false if no AprilTag detected) */
    public TagData getTagData() {
        TagData data = new TagData();

        List<AprilTagDetection> list = tagProcessor.getDetections();
        if (list == null || list.isEmpty()) return data;

        AprilTagDetection tag = list.get(0);
        if (tag.ftcPose == null) return data;

        data.hasTag = true;
        data.id = tag.id;

        // FTC coordinate frame:
        // x = left/right (inches)
        // y = forward/back (inches)
        // z = up/down (unused)
        data.forwardIn = tag.ftcPose.y;
        data.lateralIn = tag.ftcPose.x;
        data.yawDeg = tag.ftcPose.yaw;

        return data;
    }

    /** Cleanly closes the camera */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
