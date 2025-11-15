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

public class VisionSystem {

    public static class TagData {
        public boolean hasTag = false;
        public double forwardIn = 0.0;   // 朝向桶的距离（英寸）
        public double lateralIn = 0.0;   // 左右偏移（英寸）
        public double yawDeg = 0.0;      // 旋转角度（度）
    }

    private final VisionPortal visionPortal;
    private final AprilTagProcessor tagProcessor;

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

        // 等到摄像头开始推流再调曝光
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            // busy wait ok during init
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl != null && exposureControl.isExposureSupported()) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS);
        }

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(80);
        }
    }

    public TagData getTagData() {
        TagData data = new TagData();

        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections == null || detections.isEmpty()) {
            return data; // hasTag = false
        }

        AprilTagDetection tag = detections.get(0);
        if (tag.ftcPose == null) {
            return data;
        }

        // FTC 官方约定：x 左右、y 前后、z 高度；这里我们用 y 当 forward, x 当 lateral
        data.hasTag = true;
        data.forwardIn = tag.ftcPose.y;
        data.lateralIn = tag.ftcPose.x;
        data.yawDeg = tag.ftcPose.yaw;

        return data;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
