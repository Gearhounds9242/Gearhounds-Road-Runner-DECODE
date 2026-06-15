package org.firstinspires.ftc.teamcode.TeleOp;

import static com.seattlesolvers.solverslib.purepursuit.PurePursuitUtil.angleWrap;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.BilinearInterpolator;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Mechanisms.BilinearInterpolator;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


/**
 * Main TeleOp for field-centric mecanum drive + intake + shooter + drop.
 * Also includes AprilTag auto-alignment:
 * - Only adjusts lateral (strafe) and heading (yaw)
 * - Does NOT change forward/back distance
 * - Only aligns to your own goal (filter by tag ID + alliance flag)
 */
@Config
@TeleOp(name = "Mechanum", group = "TeamCode/TeleOp")
public class Mechanum extends OpMode {
    private static final double interval = 10;
    public static int TARGET_ID = 0;
    public static boolean isRedAlliance = false;   // set false when you are blue
    public static double Intake_Speed = 1000.0;
    // back power 1650 for both
    // front power 1600 bottom 1480 top
    public static double Top_Target_Speed = 2000.0;
    public static double Bottom_Target_Speed = 2000.0;
    public static double shift = 1.0;
    // Drop servo + ball count logic
    public static double rotationFactor = -0.015;
    public static double aimTolorance = 0.2;
    public static int ballNumber = 0;
    public static int goalX = 0;
    public static int goalY = 0;
    public static double offset = 0;
    public static double rightKickstandPosition;
    public static double leftKickstandPosition;
    public static double top_P = 0.0018;
    public static double top_I = 0;
    public static double top_D = 0;
    public static double top_F = 0.0004;
    public static double bottom_P = 0.0018;
    public static double bottom_I = 0;
    public static double bottom_D = 0;
    public static double bottom_F = 0.0004;
    public static double aim_P = 0.02;
    public static double aim_I = 0.025;
    public static double aim_D = 0.0001;
    public static double rightLightColor = 0;
    public static double leftLightColor = 0;
    public static double leftLightDeafualtColor = 0;
    public static double rightLightDeafualtColor = 0;
    public static double range;
    public static boolean autoPower = true;
    public static boolean driverIsControlling;
    public static double farOffset = 0;
    public static double closeOffset = 0;
    public static double bearing = 0;
    public static double MIN_TURN_POWER    = 0.1;
    public static double yaw;
    public double aimOutput = 0;
    public static double shooterTolerance;
    public static double transfer_velocity = 2000;
    static double ROLLER_VELOCITY_TOLERANCE = 65;
    private double headingOffset;
    private final GearhoundsHardware robot = new GearhoundsHardware();
    private final ElapsedTime runtime = new ElapsedTime();
    public boolean canSeeTag;
    public boolean kickstandDown = false;
    InterpLUT velocityTopLut = new InterpLUT();
    InterpLUT velocityBottomLut = new InterpLUT();
    BilinearInterpolator redOffset = new BilinearInterpolator();
    BilinearInterpolator blueOffset = new BilinearInterpolator();
    PIDFController topShooterController;
    PIDFController bottomShooterController;
    PIDController aimController;
    //    Gamepad.RumbleEffect yesEffect = new Gamepad.RumbleEffect.Builder()
//            .addStep(10.0, 0.0, 300)   // strong rumble for 0.3 sec
//            .addStep(0.0, 0.0, 150)   // pause for 0.15 sec
//            .addStep(0.0, 10.0, 300)   // strong rumble
//            .build();
    Gamepad.RumbleEffect noEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(5.0, 0.0, 80)   // strong rumble for 0.3 sec
            .addStep(0.0, 0.0, 150)   // pause for 0.15 sec
            .addStep(0.0, 5.0, 100)   // strong rumble
            .build();
    private FtcDashboard dashboard;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private WhiteBalanceControl whiteBalanceControl;
    private PtzControl ptzControl = null;
    private boolean cameraReady = false;
    private boolean controlsReady = false;
    public static long EXPOSURE_MS = 12;
    public static int GAIN = 28;
    public static int WHITE_BALANCE_K = 4000;
    public double redOffsetOutput;
    public double blueOffsetOutput;

    public static int zoom;
    private MecanumDrive drive;
    public Position cameraPosition = new Position(DistanceUnit.INCH,
            0, -3.74102362, 15.939252, 0);
    public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -75, 180, 0);


    private List<Action> driveActions = new ArrayList<>();

    @Override
    public void init() {
        robot.init(hardwareMap);             // motors/servos/IMU setup// webcam + AprilTag setup


        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
//        TARGET_ID = 20;
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        //inverse of april tag yaw - 45 degreese
        redOffset
                .add(-50,50, 0)
                .add(-25,25, 0)
                .add(-25,-25,-2)
                .add(50,-50,0)
                .add(-30,-24,0.5)
                .add(-60,-35,0.8)
                .add(-50,0,-2.9)
                .add(50,0,-2.7)
                .add(-28,-20,1.1)
                .add(52,-18,5.1)
                .add(36,1,5.1)
                .add(60,10,-8.1)
                .add(62,-19, -7)
                .add(65,17,-6)
//                .add(60,28,4.1)
                .add(41.6,-2.3,-5.7)
                .add(0,0,0);



        blueOffset
                .add(-50,-50,0)
                .add(-25,-25,0)
                .add(25,25,0)
                .add(50,50,0)
                .add(-30,25,2.1)
                .add(-40,0,3.1)
                .add(-40,20,1.1)
                .add(52,-24,2.1)
                .add(50,-20,2.67)
                .add(40,-1,2.1)
                .add(52,23,2.1)
                .add(0,0,0);


        velocityTopLut.add(-1, 0);
        velocityTopLut.add(0, 0);
        velocityTopLut.add(55, 400);
        velocityTopLut.add(65.5, 700);
        velocityTopLut.add(66,1125);
        velocityTopLut.add(74, 880);
        velocityTopLut.add(86,1125);
        velocityTopLut.add(125,1170);
        velocityTopLut.add(134.5,1241);
        velocityTopLut.add(190, 1400);
        velocityTopLut.createLUT();
//
        velocityBottomLut.add(-1, 0);
        velocityBottomLut.add(0, 0);
        velocityBottomLut.add(55, 1820);
        velocityBottomLut.add(65.5, 1480);
        velocityBottomLut.add(66,1200);
        velocityBottomLut.add(74, 1380);
        velocityBottomLut.add(86,1125);
        velocityBottomLut.add(125,1170);
        velocityBottomLut.add(134.5,1241);
        velocityBottomLut.add(190, 1400);
        velocityBottomLut.createLUT();

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
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .enableLiveView(true)
                .build();
        tagProcessor.setDecimation(1);


        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        topShooterController = new PIDFController(top_P, top_I, top_D, top_F);
        bottomShooterController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);
        aimController = new PIDController(aim_P, aim_I, aim_D);
    }


    @Override
    public void init_loop() {
        // Wait until the camera is streaming before trying to grab controls
        if (!cameraReady && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            exposureControl  = visionPortal.getCameraControl(ExposureControl.class);
            gainControl      = visionPortal.getCameraControl(GainControl.class);
            whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
            ptzControl = visionPortal.getCameraControl(PtzControl.class);

            // Switch to manual mode so our values are respected
            exposureControl.setMode(ExposureControl.Mode.Manual);
            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
            ptzControl.setZoom(zoom);

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
            telemetry.addData("Framerate", visionPortal.getFps());
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
    public void  loop() {
        // Update Road Runner’s localization
        drive.localizer.update();
        Pose2d pose = drive.localizer.getPose();
        //
        TelemetryPacket packet = new TelemetryPacket();
        redOffsetOutput = redOffset.get(pose.position.x, pose.position.y);
        blueOffsetOutput = blueOffset.get(pose.position.x, pose.position.y);

        packet.put("redOffsetOutput", redOffsetOutput);
        packet.put("blueOffsetOutput", blueOffsetOutput);

        driverIsControlling =
            Math.abs(gamepad1.left_stick_x) > 0.1 ||
            Math.abs(gamepad1.left_stick_y) > 0.1 ||
            Math.abs(gamepad1.right_stick_x) > 0.1 ||
            Math.abs(gamepad1.right_stick_y) > 0.1;

        topShooterController.setPIDF(top_P, top_I, top_D, top_F);
        bottomShooterController.setPIDF(bottom_P, bottom_I, bottom_D, bottom_F);
        aimController.setPID(aim_P,aim_I,aim_D);
        if ((range > 0 && range < 189.9) && autoPower) {
            Top_Target_Speed = velocityTopLut.get(range);
            Bottom_Target_Speed = velocityBottomLut.get(range);
        }

        double topOutput = topShooterController.calculate(robot.topMotor.getVelocity(), Top_Target_Speed);
        double bottomOutput = bottomShooterController.calculate(robot.bottomMotor.getVelocity(), Bottom_Target_Speed);
        boolean topReady = Math.abs(Top_Target_Speed - robot.topMotor.getVelocity()) < ROLLER_VELOCITY_TOLERANCE;
        boolean bottomReady = Math.abs(Bottom_Target_Speed - robot.bottomMotor.getVelocity()) < ROLLER_VELOCITY_TOLERANCE;

        double turnPower = 0;

        if ((topReady && bottomReady) && (Math.abs(bearing) < aimTolorance)) {
            leftLightColor = 0.472;
            rightLightColor = 0.472;
        } else if (topReady && bottomReady) {
            leftLightColor = 0.583;
            rightLightColor = 0.583;
        } else if (Math.abs(bearing) < offset) {
            leftLightColor = 0.388;
            rightLightColor = 0.388;
        } else {
            // Neither ready
            leftLightColor = 0;
            rightLightColor = 0;
        }



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



//        if (gamepad1.left_bumper) shift = 0.3; // slow mode
//        if (gamepad1.right_bumper) shift = 1.0; // full speed

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


//if (autoPower) {
//    if (!isRedAlliance) {
//        offset = blueOffset.get(pose.position.x, pose.position.y);
//    }
//
//    if (isRedAlliance) {
//        offset = redOffset.get(pose.position.x, pose.position.y);
//    }
//}
        if (gamepad1.ps && isRedAlliance){
            drive.localizer.setPose(new Pose2d(60.5, -61, Math.toRadians(90)));
        }
        if (gamepad1.ps && !isRedAlliance){
            drive.localizer.setPose(new Pose2d(60.5, 61, Math.toRadians(90)));
        }


        if (gamepad1.dpadUpWasPressed()){
            offset += 1;
        }
        if (gamepad1.dpadDownWasPressed()){
            offset -= 1;
        }



       if (gamepad1.touchpadWasPressed() && kickstandDown == true){
           robot.rightKickstand.setPosition(1);
           robot.leftKickstand.setPosition(0);
           kickstandDown = false;
       }
       if(gamepad1.shareWasPressed() && kickstandDown == false){
           robot.rightKickstand.setPosition(0.8);
           robot.leftKickstand.setPosition(0.2);
           kickstandDown = true;
       }

        if (gamepad1.right_trigger > 0.1) {
            robot.intake.setPower(1);
        }
        else {
            robot.intake.setPower(0);
        }
        if (gamepad1.y){
            robot.intake.setPower(-1);
        }

        if (Math.abs(gamepad2.right_trigger) > 0.1) {
            robot.bottomMotor.setPower(bottomOutput * 12/robot.voltageSensor.getVoltage());
        } else {
            robot.bottomMotor.setPower(0.0);
        }

        if (Math.abs(gamepad2.left_trigger) > 0.1) {
            robot.topMotor.setPower(topOutput * 12/robot.voltageSensor.getVoltage());
        } else {
            robot.topMotor.setPower(0.0);
        }
        if (gamepad2.dpadUpWasPressed() && !autoPower) {
            Top_Target_Speed += interval;
        }
        if (gamepad2.dpadDownWasPressed() && !autoPower) {
            Top_Target_Speed -= interval;
        }

        if (gamepad2.dpadRightWasPressed() && !autoPower) {
            Bottom_Target_Speed += interval;
        }
        if (gamepad2.dpadLeftWasPressed() && !autoPower) {
            Bottom_Target_Speed -= interval;
        }

        if (gamepad2.x && !autoPower) {
            Top_Target_Speed = 1120;
            Bottom_Target_Speed = 1120;
        }

        if (gamepad2.b && !autoPower) {
            Top_Target_Speed = 1190;
            Bottom_Target_Speed = 1190;

        }


//        if(gamepad2.b && autoPower == true){
//            offset = farOffset;
//        }
//        if(gamepad2.x && autoPower == true){
//            offset = closeOffset;
//
//        }
        if (gamepad2.right_bumper) {
            robot.transfer.setVelocity(transfer_velocity);
            robot.intake.setPower(1);
        }

        if (gamepad2.left_bumper || gamepad2.a) {
            robot.transfer.setPower(-transfer_velocity);
        } else {
            robot.transfer.setPower(0);
        }

        if (gamepad2.touchpadWasPressed() && autoPower == true) {
            autoPower = false;
            leftLightDeafualtColor = 0.333;
            rightLightDeafualtColor = 0.333;
        }
        if (gamepad2.psWasPressed() && autoPower == false) {
            autoPower = true;
            leftLightDeafualtColor = 0;
            rightLightDeafualtColor = 0;
        }


/*
CAMERA STUFF
 */

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
            yaw = targetTag.ftcPose.yaw;
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
                    telemetry.addData("cameraOutputY",cameraOutputY);
                    telemetry.addData("cameraOutputYaw", cameraOutputYaw);
                    packet.put("cameraOutputX", cameraOutputX);
                    packet.put("cameraOutputY", cameraOutputY);
                    packet.put("cameraOutputYaw", cameraOutputYaw);

                    if(range <= 100) {
                        drive.localizer.setPose(new Pose2d(cameraOutputX, cameraOutputY, (Math.toRadians(cameraOutputYaw + 90))));
                    }
                }
            }
        }



        if (gamepad1.left_trigger > 0.9) {
            if (targetTag != null) {

                double robotX = pose.position.x;
                double robotY = pose.position.y;
                double dx = goalX - robotX;
                double dy = goalY - robotY;
                double robotHeading = pose.heading.toDouble();
                double robotHeadingDegrees = Math.toDegrees(robotHeading);
                double fieldHeadingDegrees = Math.toDegrees(Math.atan2(dy, dx));
                double targetHeading = fieldHeadingDegrees - robotHeadingDegrees;
                packet.put("robotHeading", robotHeading);
                packet.put("targetHeading", targetHeading);
                // Tag visible: old controll that I know works
                double bearing = targetTag.ftcPose.bearing;
                while (targetHeading >  Math.PI) targetHeading -= 2 * Math.PI;
                while (targetHeading < -Math.PI) targetHeading += 2 * Math.PI;

                turnPower = aimController.calculate(robotHeading, targetHeading + offset);

                if (Math.abs(turnPower) < MIN_TURN_POWER && Math.abs(bearing) > aimTolorance) {
                    turnPower = Math.copySign(MIN_TURN_POWER, turnPower);
                }

                telemetry.addData("range", targetTag.ftcPose.range);
                telemetry.addData("yaw", yaw);

            }
        }else{
            turnPower = 0;

        }//bearing to bearing offset

        // if driver not pressing the trigger make the aimOutput power = 0
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
                                            (-gamepad1.left_stick_y * shift),
                                            (-gamepad1.left_stick_x * shift)
                                    ),
                                    (-gamepad1.right_stick_x * shift) + turnPower
                            ))
            );
        }



//        packet.put("TopMotorRPM", (robot.TopMotor.getVelocity() / 28.0) * 60.0);
//        packet.put("BottomMotorRPM", (robot.BottomMotor.getVelocity() / 28.0) * 60.0);
//        packet.put("TopCurrentA", robot.TopMotor.getCurrent(CurrentUnit.AMPS));
//        packet.put("BottomCurrentA", robot.BottomMotor.getCurrent(CurrentUnit.AMPS));
//        packet.put("IntakeRPM", (robot.intake.getVelocity() / 28) * 60);
        packet.put("bearing", bearing);
        packet.put("offset", offset);
        packet.put("BottomVelocity", robot.bottomMotor.getVelocity());
        packet.put("TopVelocity", robot.topMotor.getVelocity());
        packet.put("TopTarget", Top_Target_Speed);
        packet.put("BottomTarget", Bottom_Target_Speed);
        packet.fieldOverlay().setStroke("#3F51B5");
        packet.fieldOverlay().setFill("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        dashboard.sendTelemetryPacket(packet);
//        telemetry.addData("Intake Power", Intake_Speed);

        telemetry.addData("goalX", goalX);
        telemetry.addData("goalY", goalY);
        telemetry.addData("Top Shooter Target Speed", Top_Target_Speed);
        telemetry.addData("Bottom Shooter Target Speed", Bottom_Target_Speed);
        telemetry.addData("Selected Id", TARGET_ID);
//        telemetry.addData("bearing", bearing);
        telemetry.addData("range", range);
        telemetry.addData("autopower", autoPower);
        telemetry.addData("X",drive.localizer.getPose().position.x);
        telemetry.addData("Y",drive.localizer.getPose().position.y);
        telemetry.addData("Heading",Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("offset", offset);
        telemetry.addData("transfer", robot.transfer.getCurrentPosition());
        telemetry.addData("blueOffsetOutput", blueOffsetOutput);
        telemetry.addData("redOffsetOutput", redOffsetOutput);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

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

//        // Read the camera's supported ranges
//        minExposure = exposureControl.getMinExposure(TimeUnit.MILLISECONDS);
//        maxExposure = exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
//        minGain = gainControl.getMinGain();
//        maxGain = gainControl.getMaxGain();
//        minWhiteBalance = whiteBalanceControl.getMinWhiteBalanceTemperature();
//        maxWhiteBalance = whiteBalanceControl.getMaxWhiteBalanceTemperature();
//
//        // Initialize current values from the camera's current state
//        curExposure = exposureControl.getExposure(TimeUnit.MILLISECONDS);
//        curGain = gainControl.getGain();
//        curWhiteBalance = whiteBalanceControl.getWhiteBalanceTemperature();
//
//        // Sync the Dashboard @Config fields
//        EXPOSURE_MS = curExposure;
//        GAIN = curGain;
//        WHITE_BALANCE_K = curWhiteBalance;
    }

}