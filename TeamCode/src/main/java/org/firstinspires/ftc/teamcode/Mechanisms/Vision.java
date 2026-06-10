package org.firstinspires.ftc.teamcode.Mechanisms;

import android.util.Size;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class Vision {

    // Tuning constants - all adjustable from FTC Dashboard
    public static double aimTolorance = 3.0;
    public static double rotationFactor = -0.03;
//    public static double MAX_TURN_POWER        = 0.4;
//    public static double MIN_TURN_POWER        = 0.05;
    public static double ALIGN_TIMEOUT_SEC     = 3;
    public static int    STABLE_COUNT_REQ      = 5;
    public Position cameraPosition = new Position(DistanceUnit.INCH,
            0, -3.74102362, 15.939252, 0);
    public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -75, 180, 0);

    // Tag IDs
    public static final int RED_GOAL_TAG_ID  = 24;
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static long EXPOSURE_MS = 6;
    public static int GAIN = 0;

    private final GearhoundsHardware robot;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor tagProcessor;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private WhiteBalanceControl whiteBalanceControl;
    private PtzControl ptzControl = null;
    private MecanumDrive drive;

    public Vision(GearhoundsHardware robot) {
        this.robot = robot;
        WebcamName webcam = robot.webcam;

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.36479005)
                .setCameraPose(cameraPosition,cameraOrientation)
                .build();

        // Vision portal
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .enableLiveView(true)
                .build();
    }

    /**
     * MUST be called after MecanumDrive is initialized, before using alignToTag()
     */
    public void setDrive(MecanumDrive drive) {
        this.drive = drive;
//        exposureControl  = visionPortal.getCameraControl(ExposureControl.class);
//        gainControl      = visionPortal.getCameraControl(GainControl.class);
//
//        // Switch to manual mode so our values are respected
//        exposureControl.setMode(ExposureControl.Mode.Manual);
//        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
//
//        // Apply initial values
//        exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
//        gainControl.setGain(GAIN);
    }

    /**
     * Returns an Action that rotates the robot until it is aimed at the target tag.
     *
     * @param targetTagId  RED_GOAL_TAG_ID or BLUE_GOAL_TAG_ID
     * @param offset       Offset in degrees - positive shifts aim left of tag center
     */
    public Action alignToTag(int targetTagId, double offset) {
        if (drive == null) {
            throw new RuntimeException("Vision.setDrive() must be called before alignToTag()!");
        }
        return new AlignToTag(targetTagId, offset);
    }

    public AprilTagDetection getTag(int targetTagId) {
        List<AprilTagDetection> tags = tagProcessor.getDetections();
        for (AprilTagDetection t : tags) {
            if (t.id == targetTagId) return t;
        }
        return null;
    }

    public Action close() {
        return new Vision.close();
    }

    // ---------------------------------------------------------------
    // Inner Action
    // ---------------------------------------------------------------
    private class AlignToTag implements Action {
        private final int    targetTagId;
        private final double offset;
        private ElapsedTime  timer;
        private int          stableCount = 0;
        private int tagNotVisibleItCount = 0;
        private int itNumber = 0;
        private ExposureControl exposureControl;
        private GainControl gainControl;
        private WhiteBalanceControl whiteBalanceControl;
        private PtzControl ptzControl = null;
        private long EXPOSURE_MS = 6;
        private int GAIN = 0;

        AlignToTag(int targetTagId, double offset) {
            this.targetTagId = targetTagId;
            this.offset      = offset;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            itNumber++;
            drive.localizer.update();
            AprilTagDetection target = getTag(targetTagId);


            if (timer == null) {
                timer = new ElapsedTime();
            }
            // Hard timeout
            if (timer.seconds() > ALIGN_TIMEOUT_SEC) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                packet.addLine("AlignToTag: TIMED OUT");
                return false;
            }

            if (visionPortal.getFps() == 0){
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                packet.addLine("AlignToTag: CAMERA SNAPSHOT");
                return  false;
            }


            if (target == null){
                tagNotVisibleItCount++;
            }
            if (tagNotVisibleItCount > 20) {
                packet.addLine("AlignToTag: Not Visible count exceeded 20");
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                return false;
            }

            packet.put("AlignToTag: IsAttached", robot.webcam.isAttached());
            packet.put("AlignToTag: CameraName", robot.webcam.getUsbDeviceNameIfAttached());

            if (target != null) {
                tagNotVisibleItCount = 0;
                packet.put("AlignToTag bearing", target.ftcPose.bearing);
                packet.put("AlignToTag range", target.ftcPose.range);
                packet.put("AlignToTag stableCount", stableCount);



                // Check tolerance
                // bearing is how many degrees the tag is left/right of camera center
                // offset shifts where we want to be relative to tag center
                if (Math.abs(target.ftcPose.bearing + offset) < aimTolorance) {
                    stableCount++;
                    if (stableCount > STABLE_COUNT_REQ) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        packet.addLine("AlignToTag: LOCKED");
                        return false;
                    }
                } else {
                    stableCount = 0;
                    // Proportional turn toward tag - negated because positive bearing
                    // means tag is to the left, so we turn left (positive angular vel)
                    double turnPower = target.ftcPose.bearing * rotationFactor;

                    packet.put("AlignToTag turnPower", turnPower);

                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
                }



            }

//            List<AprilTagDetection> tags = tagProcessor.getDetections();
//            packet.put("List Length", tags.toArray().length);
//            for (AprilTagDetection t : tags) {
//                packet.put("Detections", t.id);
//
//                if(t.id == RED_GOAL_TAG_ID){
//                    packet.put("it number", itNumber);
//                    return false;
//                }
//            }

            return true;
        }
    }

    public class close implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            visionPortal.close();
            return false;

        }
    }
}