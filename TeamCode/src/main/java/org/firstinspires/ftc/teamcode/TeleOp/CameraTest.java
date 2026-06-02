package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name = "CameraTest", group = "TeamCode/TeleOp")
public class CameraTest extends OpMode {
    public static int TARGET_ID = 0;
    public static boolean isRedAlliance = false;   // set false when you are blue
    public static int goalX = 0;
    public static int goalY = 0;
    public static double offset = 0;
    public static double rightLightColor = 0;
    public static double leftLightColor = 0;
    public static double leftLightDeafualtColor = 0;
    public static double rightLightDeafualtColor = 0;
    public static double range;
    public static boolean autoPower = true;
    public static boolean driverIsControlling;
    public static double bearing = 0;
    public static long EXPOSURE_MS = 6;
    public static int GAIN = 0;
    public static int WHITE_BALANCE_K = 4000;
    private final GearhoundsHardware robot = new GearhoundsHardware();
    private final ElapsedTime runtime = new ElapsedTime();
    public boolean canSeeTag;
    public boolean kickstandDown = false;
    public Position cameraPosition = new Position(DistanceUnit.INCH,
            0, -3.74102362, 15.939252, 0);
    public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -75, 180, 0);
    private double headingOffset;
    private FtcDashboard dashboard;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private WhiteBalanceControl whiteBalanceControl;
    private boolean cameraReady = false;
    private MecanumDrive drive;
    private final List<Action> driveActions = new ArrayList<>();

    // Track previous values so we only write to the camera when something changes
    private long prevExposure = -1;
    private int prevGain = -1;
    private int prevWhiteBalance = -1;

    @Override
    public void init() {
        robot.init(hardwareMap);             // motors/servos/IMU setup// webcam + AprilTag setup


        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
//        TARGET_ID = 20;
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.36479005)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        // Vision portal
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .enableLiveView(true)
                .build();


        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        dashboard.startCameraStream(visionPortal, 0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }


    @Override
    public void init_loop() {
        // Wait until the camera is streaming before trying to grab controls
        if (!cameraReady && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            exposureControl  = visionPortal.getCameraControl(ExposureControl.class);
            gainControl      = visionPortal.getCameraControl(GainControl.class);
            whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

            // Switch to manual mode so our values are respected
            exposureControl.setMode(ExposureControl.Mode.Manual);
            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);

            // Apply initial values
            exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
            gainControl.setGain(GAIN);
            whiteBalanceControl.setWhiteBalanceTemperature(WHITE_BALANCE_K);

            cameraReady = true;
        }

        if (cameraReady) {
            telemetry.addLine("Camera ready");
            telemetry.addData("Exposure", EXPOSURE_MS);
            telemetry.addData("Gain", GAIN);
            telemetry.addData("White Balance", WHITE_BALANCE_K);
        } else {
            telemetry.addLine("Waiting for camera");
        }
    }

    @Override
    public void start() {
        runtime.reset();
//        if(!gamepad1.a || !gamepad2.a){
//        robot.block.setPosition(0.45);
//        }
    }


    @Override
    public void loop() {
        // Update Road Runner's localization
        drive.localizer.update();
        Pose2d pose = drive.localizer.getPose();
        TelemetryPacket packet = new TelemetryPacket();
        driverIsControlling =
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                        Math.abs(gamepad1.left_stick_y) > 0.1 ||
                        Math.abs(gamepad1.right_stick_x) > 0.1 ||
                        Math.abs(gamepad1.right_stick_y) > 0.1;


        double turnPower = 0;


        if (leftLightColor >= 0.001) {
            robot.leftLight.setPosition(leftLightColor);
        }
        if (leftLightColor <= 0) {
            robot.leftLight.setPosition(leftLightDeafualtColor);
        }
        if (rightLightColor >= 0.001) {
            robot.rightLight.setPosition(rightLightColor);
        }
        if (rightLightColor <= 0) {
            robot.rightLight.setPosition(rightLightDeafualtColor);
        }


        if (gamepad1.dpadRightWasPressed() || gamepad2.optionsWasPressed()) {
            TARGET_ID = 24;
            offset = 2.1;
            goalX = -70;
            goalY = 70;
            isRedAlliance = true;
        }
        if (gamepad1.dpadLeftWasPressed() || gamepad2.shareWasPressed()) {
            TARGET_ID = 20;
            goalX = -70;
            goalY = -70;
            isRedAlliance = false;
        }


        if (gamepad1.ps && isRedAlliance) {
            drive.localizer.setPose(new Pose2d(60.5, -61, Math.toRadians(90)));
        }
        if (gamepad1.ps && !isRedAlliance) {
            drive.localizer.setPose(new Pose2d(60.5, 61, Math.toRadians(90)));
        }


        if (gamepad2.touchpadWasPressed() && autoPower) {
            autoPower = false;
            leftLightDeafualtColor = 0.333;
            rightLightDeafualtColor = 0.333;
        }
        if (gamepad2.psWasPressed() && !autoPower) {
            autoPower = true;
            leftLightDeafualtColor = 0;
            rightLightDeafualtColor = 0;
        }


/*
CAMERA STUFF
 */

        // ---- Live camera control from FTC Dashboard ----
        // Only write when a Dashboard value actually changes to avoid
        // hammering the USB camera control interface every loop.
        if (cameraReady) {
            if (EXPOSURE_MS != prevExposure) {
                exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
                prevExposure = EXPOSURE_MS;
            }
            if (GAIN != prevGain) {
                gainControl.setGain(GAIN);
                prevGain = GAIN;
            }
            if (WHITE_BALANCE_K != prevWhiteBalance) {
                whiteBalanceControl.setWhiteBalanceTemperature(WHITE_BALANCE_K);
                prevWhiteBalance = WHITE_BALANCE_K;
            }
        }

        // Get detections
        List<AprilTagDetection> detections = tagProcessor.getDetections();

        // Find ONLY the target ID
        AprilTagDetection targetTag = null;

        for (AprilTagDetection goalTag : detections) {
            if (goalTag.id == TARGET_ID) {
                targetTag = goalTag;
                break;
            }
        }
        if (targetTag != null) {
            range = targetTag.ftcPose.range;
            bearing = targetTag.ftcPose.bearing;
        } else {
            range = -1;
            bearing = 0;
            canSeeTag = false;
        }

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    double cameraOutputX = detection.robotPose.getPosition().x;
                    double cameraOutputY = detection.robotPose.getPosition().y;
//                    double cameraOutputZ = detection.robotPose.getPosition().z;

//                    double cameraOutputPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
//                    double cameraOutputRoll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
                    double cameraOutputYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                    telemetry.addData("cameraOutputX", cameraOutputX);
                    telemetry.addData("cameraOutputY", cameraOutputY);
                    telemetry.addData("cameraOutputYaw", cameraOutputYaw);
                    packet.put("cameraOutputX", cameraOutputX);
                    packet.put("cameraOutputY", cameraOutputY);
                    packet.put("cameraOutputYaw", cameraOutputYaw);

                    drive.localizer.setPose(new Pose2d(cameraOutputX, cameraOutputY, (Math.toRadians(cameraOutputYaw + 90))));

                }
            }
        }


        if (gamepad1.optionsWasPressed()) {
            headingOffset = drive.localizer.getPose().heading.toDouble();
        }

        // ================================================================
        // DRIVE CONTROL
        //
        // If a drive action is running, skip manual input so we don't
        // fight the action. The driver can always cancel by moving a stick.
        // If no drive action is running, normal field-centric drive applies.
        // ================================================================
        if (driveActions.isEmpty()) {
            drive.setDrivePowers(
                    Rotation2d.exp(-drive.localizer.getPose().heading.toDouble() + headingOffset)
                            .times(new PoseVelocity2d(
                                    new Vector2d(
                                            (-gamepad1.left_stick_y),
                                            (-gamepad1.left_stick_x)
                                    ),
                                    (-gamepad1.right_stick_x) + turnPower
                            ))
            );
        }


        packet.put("BottomVelocity", robot.bottomMotor.getVelocity());
        packet.put("TopVelocity", robot.topMotor.getVelocity());
        packet.put("Exposure", EXPOSURE_MS);
        packet.put("Gain", GAIN);
        packet.put("WhiteBalance", WHITE_BALANCE_K);
        packet.fieldOverlay().setStroke("#3F51B5");
        packet.fieldOverlay().setFill("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Selected Id", TARGET_ID);
//        telemetry.addData("bearing", bearing);
        telemetry.addData("range", range);
        telemetry.addData("autopower", autoPower);
        telemetry.addData("X", drive.localizer.getPose().position.x);
        telemetry.addData("Y", drive.localizer.getPose().position.y);
        telemetry.addData("Heading", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("offset", offset);
        telemetry.addData("Exposure", EXPOSURE_MS);
        telemetry.addData("Gain", GAIN);
        telemetry.addData("WhiteBalance", WHITE_BALANCE_K);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }


}