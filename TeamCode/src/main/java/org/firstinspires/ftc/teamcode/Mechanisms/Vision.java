package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

/**
 * VisionSystem: shared camera + AprilTag utility for all autonomous and teleop modes.
 *
 * Usage in autonomous:
 *   VisionSystem vision = new VisionSystem(hardwareMap);
 *   vision.setDrive(drive);
 *   ...
 *   .stopAndAdd(vision.alignToTag(VisionSystem.RED_GOAL_TAG_ID))
 *   ...
 *   vision.close();
 */
public class Vision {

    // --- Tuning constants: adjust here and all autos update automatically ---
    public static final double BEARING_TOLERANCE_DEG = 1.5;
    public static final double TURN_GAIN             = 0.015;
    public static final double ALIGN_TIMEOUT_SEC     = 2.0;

    // --- Tag IDs ---
    public static final int RED_GOAL_TAG_ID  = 24;
    public static final int BLUE_GOAL_TAG_ID = 20;

    private final GearhoundsHardware robot;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor tagProcessor;
    private MecanumDrive drive;

    /**
     * Builds the camera pipeline and blocks until streaming begins.
     * Call this before waitForStart() in your OpMode.
     */
    public Vision(GearhoundsHardware robot) {
        this.robot = robot;
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

//        // Block until streaming so first detections are valid
//        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            try { Thread.sleep(20); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
//        }
//
//        // Manual exposure for consistent tag detection under field lighting
//        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//        if (exposureControl != null && exposureControl.isExposureSupported()) {
//            exposureControl.setMode(ExposureControl.Mode.Manual);
//            exposureControl.setExposure(15, TimeUnit.MILLISECONDS);
//        }
//
//        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//        if (gainControl != null) {
//            gainControl.setGain(80);
//        }
    }

    /**
     * Must be called after MecanumDrive is initialized, before using alignToTag().
     */
    public void setDrive(MecanumDrive drive) {
        this.drive = drive;
    }

    /**
     * Returns an Action that rotates the robot in place until it is aimed at the target tag.
     * - If the tag is not visible, exits immediately (odometry got us close enough)
     * - Has a hard timeout so auto never hangs
     *
     * @param targetTagId  Use VisionSystem.RED_GOAL_TAG_ID or VisionSystem.BLUE_GOAL_TAG_ID
     * @param offset Offset in degrees from the aprilTag
     */
    public Action alignToTag(int targetTagId,double offset) {
        return new AlignToTag(targetTagId, offset);
    }

    /**
     * Returns the current detection for a given tag ID, or null if not visible.
     */
    public AprilTagDetection getTag(int targetTagId) {
        List<AprilTagDetection> tags = tagProcessor.getDetections();
        for (AprilTagDetection t : tags) {
            if (t.id == targetTagId) return t;
        }
        return null;
    }

    /**
     * Always call this at the end of your OpMode to release the camera.
     */
    public void close() {
        if (visionPortal != null) visionPortal.close();
    }

    // ---------------------------------------------------------------
    // Inner Action — used by alignToTag()
    // ---------------------------------------------------------------
    private class AlignToTag implements Action {
        private final int targetTagId;
        private final double offset;
        ElapsedTime timer;

        AlignToTag(int targetTagId, double offset) {
            this.targetTagId = targetTagId;
            this.offset = offset;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            drive.localizer.update();

            if (timer == null) {
                timer = new ElapsedTime();
            }

            // Hard timeout - never block longer than ALIGN_TIMEOUT_SEC
            if (timer.seconds() > ALIGN_TIMEOUT_SEC) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                packet.addLine("AlignToTag: timed out");
                return false;
            }

            AprilTagDetection target = getTag(targetTagId);

            // Tag not visible - odometry got us close enough, just continue
            if (target == null) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                packet.addLine("AlignToTag: tag not visible, skipping");
                return false;
            }

            double bearing = target.ftcPose.bearing;
            packet.put("AlignBearing", bearing);

            // Within tolerance - locked on
            if ((Math.abs(bearing) + offset) < BEARING_TOLERANCE_DEG) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                packet.addLine("AlignToTag: LOCKED");
                return false;
            }

            // Rotate proportionally toward the tag
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0, 0),
                    bearing * TURN_GAIN
            ));
            return true;
        }
    }
}