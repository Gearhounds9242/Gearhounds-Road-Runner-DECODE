package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "CameraTuner", group = "Tuners")
public class CameraTuner extends OpMode {

    // ====== FTC Dashboard @Config fields ======
    // These show up in the FTC Dashboard under the "CameraTuner" config group.
    // You can live-edit them from the dashboard while the OpMode runs.
    public static long EXPOSURE_MS = 6;
    public static int GAIN = 0;
    public static int WHITE_BALANCE_K = 4000;
    public static boolean USE_DASHBOARD_VALUES = false;

    // ====== Hardware and vision ======
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    private ExposureControl exposureControl;
    private GainControl gainControl;
    private WhiteBalanceControl whiteBalanceControl;

    // Camera pose (same as your original)
    public Position cameraPosition = new Position(DistanceUnit.INCH,
            0, -3.74102362, 15.939252, 0);
    public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -75, 180, 0);

    // ====== Internal state ======
    private long curExposure;
    private int curGain;
    private int curWhiteBalance;

    private long minExposure, maxExposure;
    private int minGain, maxGain;
    private int minWhiteBalance, maxWhiteBalance;

    private boolean controlsReady = false;
    private boolean lockedIn = false;

    // Debounce timers so holding a button doesn't spam changes
    private final ElapsedTime exposureTimer = new ElapsedTime();
    private final ElapsedTime gainTimer = new ElapsedTime();
    private final ElapsedTime wbTimer = new ElapsedTime();
    private static final double scrollInterval = 0.15; // seconds between repeats

    // Previous button states for edge detection
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;

    @Override
    public void init() {
        // Merge DS telemetry with Dashboard telemetry so both show the same data
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.36479005)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        // Stream the camera preview to FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        telemetry.addLine("CameraTuner initializing...");
        telemetry.addLine("Waiting for camera to start streaming.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Wait until the camera is streaming before trying to grab controls
        if (!controlsReady && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            setupCameraControls();
            controlsReady = true;
        }

        if (controlsReady) {
            telemetry.addLine(">> Camera ready! Press START to begin tuning.");
            telemetry.addData("Exposure range", "%d - %d ms", minExposure, maxExposure);
            telemetry.addData("Gain range", "%d - %d", minGain, maxGain);
            telemetry.addData("White Balance range", "%d - %d K", minWhiteBalance, maxWhiteBalance);
        } else {
            telemetry.addLine("Waiting for camera...");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!controlsReady) {
            telemetry.addLine("Camera controls not ready yet.");
            telemetry.update();
            return;
        }

        // -------- Input handling --------
        if (USE_DASHBOARD_VALUES) {
            // Pull values from Dashboard @Config fields
            curExposure = clamp(EXPOSURE_MS, minExposure, maxExposure);
            curGain = clamp((int) GAIN, minGain, maxGain);
            curWhiteBalance = clamp(WHITE_BALANCE_K, minWhiteBalance, maxWhiteBalance);
        } else {
            // Gamepad controls
            handleGamepadInput();
        }

        // -------- Apply to camera --------
        exposureControl.setExposure(curExposure, TimeUnit.MILLISECONDS);
        gainControl.setGain(curGain);
        whiteBalanceControl.setWhiteBalanceTemperature(curWhiteBalance);

        // Keep Dashboard @Config fields in sync with current values
        EXPOSURE_MS = curExposure;
        GAIN = curGain;
        WHITE_BALANCE_K = curWhiteBalance;

        // -------- Lock-in (press Y to snapshot current values) --------
        boolean curY = gamepad1.y;
        if (curY && !prevY) {
            lockedIn = true;
        }
        prevY = curY;

        // -------- Telemetry --------
        telemetry.addLine("=== CAMERA TUNER ===");
        telemetry.addLine();

        telemetry.addData("Mode", USE_DASHBOARD_VALUES ? "DASHBOARD" : "GAMEPAD");
        telemetry.addLine();

        telemetry.addLine("--- Current Values ---");
        telemetry.addData("Exposure", "%d ms  (range: %d - %d)", curExposure, minExposure, maxExposure);
        telemetry.addData("Gain", "%d  (range: %d - %d)", curGain, minGain, maxGain);
        telemetry.addData("White Balance", "%d K  (range: %d - %d)", curWhiteBalance, minWhiteBalance, maxWhiteBalance);
        telemetry.addLine();

        // Show AprilTag detections
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        telemetry.addData("AprilTags detected", detections.size());
        for (AprilTagDetection det : detections) {
            if (det.metadata != null) {
                telemetry.addData("  Tag " + det.id, "%.1f\" away, bearing %.1f deg",
                        det.ftcPose.range, det.ftcPose.bearing);
            } else {
                telemetry.addData("  Tag " + det.id, "(unknown tag, no metadata)");
            }
        }
        telemetry.addLine();

        // Show locked-in values if the user pressed Y
        if (lockedIn) {
            telemetry.addLine("--- LOCKED-IN VALUES (copy these) ---");
            telemetry.addData("  Exposure", "%d", curExposure);
            telemetry.addData("  Gain", "%d", curGain);
            telemetry.addData("  White Balance", "%d", curWhiteBalance);
            telemetry.addLine();
        }

        telemetry.addLine("--- Gamepad Controls ---");
        telemetry.addLine("DPad Up/Down     = Exposure +/- 1 ms");
        telemetry.addLine("Bumpers L/R      = Gain -/+ 1");
        telemetry.addLine("Triggers L/R     = White Balance -/+ 100 K");
        telemetry.addLine("A                = Toggle Dashboard/Gamepad mode");
        telemetry.addLine("B                = Reset to defaults");
        telemetry.addLine("X                = Reset lock-in");
        telemetry.addLine("Y                = Lock in current values");

        telemetry.update();
    }

    // ====================================================================
    // Gamepad input with debounce
    // ====================================================================
    private void handleGamepadInput() {

        // --- Exposure: DPad Up / Down ---
        if (gamepad1.dpad_up && exposureTimer.seconds() > scrollInterval) {
            curExposure = clamp(curExposure + 1, minExposure, maxExposure);
            exposureTimer.reset();
        }
        if (gamepad1.dpad_down && exposureTimer.seconds() > scrollInterval) {
            curExposure = clamp(curExposure - 1, minExposure, maxExposure);
            exposureTimer.reset();
        }

        // --- Gain: Left Bumper (down) / Right Bumper (up) ---
        if (gamepad1.right_bumper && gainTimer.seconds() > scrollInterval) {
            curGain = clamp(curGain + 1, minGain, maxGain);
            gainTimer.reset();
        }
        if (gamepad1.left_bumper && gainTimer.seconds() > scrollInterval) {
            curGain = clamp(curGain - 1, minGain, maxGain);
            gainTimer.reset();
        }

        // --- White Balance: Left Trigger (down) / Right Trigger (up) ---
        if (gamepad1.right_trigger > 0.3 && wbTimer.seconds() > scrollInterval) {
            curWhiteBalance = clamp(curWhiteBalance + 100, minWhiteBalance, maxWhiteBalance);
            wbTimer.reset();
        }
        if (gamepad1.left_trigger > 0.3 && wbTimer.seconds() > scrollInterval) {
            curWhiteBalance = clamp(curWhiteBalance - 100, minWhiteBalance, maxWhiteBalance);
            wbTimer.reset();
        }

        // --- A: Toggle dashboard mode (edge-triggered) ---
        boolean curA = gamepad1.a;
        if (curA && !prevA) {
            USE_DASHBOARD_VALUES = !USE_DASHBOARD_VALUES;
        }
        prevA = curA;

        // --- B: Reset to defaults (edge-triggered) ---
        boolean curB = gamepad1.b;
        if (curB && !prevB) {
            curExposure = clamp(6, minExposure, maxExposure);
            curGain = minGain;
            curWhiteBalance = 4000;
        }
        prevB = curB;

        // --- X: Clear lock-in (edge-triggered) ---
        boolean curX = gamepad1.x;
        if (curX && !prevX) {
            lockedIn = false;
        }
        prevX = curX;
    }

    // ====================================================================
    // Camera control setup
    // ====================================================================
    private void setupCameraControls() {
        // Get control interfaces from VisionPortal
        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        gainControl = visionPortal.getCameraControl(GainControl.class);
        whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        // Switch exposure to manual mode (required to set gain as well)
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }

        // Switch white balance to manual mode
        if (whiteBalanceControl.getMode() != WhiteBalanceControl.Mode.MANUAL) {
            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
        }

        // Read the camera's supported ranges
        minExposure = exposureControl.getMinExposure(TimeUnit.MILLISECONDS);
        maxExposure = exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();
        minWhiteBalance = whiteBalanceControl.getMinWhiteBalanceTemperature();
        maxWhiteBalance = whiteBalanceControl.getMaxWhiteBalanceTemperature();

        // Initialize current values from the camera's current state
        curExposure = exposureControl.getExposure(TimeUnit.MILLISECONDS);
        curGain = gainControl.getGain();
        curWhiteBalance = whiteBalanceControl.getWhiteBalanceTemperature();

        // Sync the Dashboard @Config fields
        EXPOSURE_MS = curExposure;
        GAIN = curGain;
        WHITE_BALANCE_K = curWhiteBalance;
    }

    // ====================================================================
    // Utility
    // ====================================================================
    private long clamp(long value, long min, long max) {
        return Math.max(min, Math.min(max, value));
    }

    private int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}