package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


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
    public static int TARGET_ID = 0;
    public static boolean isRedAlliance = false;   // set false when you are blue
    public static double Intake_Speed = 1000.0;
    // back power 1650 for both
    // front power 1600 bottom 1480 top
    public static double Top_Target_Speed = 2000.0;
    public static double Bottom_Target_Speed = 2000.0;
    public static double shift = 1.0;
    // Drop servo + ball count logic
    public static double rotationFactor = 0.05;
    public static double aimTolorance = 1;
    public static int ballNumber = 0;
    public static double offset = 0;
    public static double top_P = 3;
    public static double top_I = 0;
    public static double top_D = 0.3;
    public static double top_F = 2.8;
    public static double bottom_P = 1;
    public static double bottom_I = 0;
    public static double bottom_D = 3;
    public static double bottom_F = 1;
    public static double rightLightColor = 0;
    public static double leftLightColor = 0;
    public static double leftLightDeafualtColor = 1;
    public static double rightLightDeafualtColor = 1;
    public static double range;
    public static double robotRange;
    public static boolean autoPower = true;
    public static double bearing = 0;
    public static double shooterTolerance;
    private static final double interval = 10;
    private final GearhoundsHardware robot = new GearhoundsHardware();
    private final ElapsedTime runtime = new ElapsedTime();
    public boolean canSeeTag;
    InterpLUT velocityTopLut = new InterpLUT();
    InterpLUT velocityBottomLut = new InterpLUT();
    Gamepad.RumbleEffect yesEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(10.0, 0.0, 300)   // strong rumble for 0.3 sec
            .addStep(0.0, 0.0, 150)   // pause for 0.15 sec
            .addStep(0.0, 10.0, 300)   // strong rumble
            .build();
    Gamepad.RumbleEffect noEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(5.0, 0.0, 80)   // strong rumble for 0.3 sec
            .addStep(0.0, 0.0, 150)   // pause for 0.15 sec
            .addStep(0.0, 5.0, 100)   // strong rumble
            .build();
    private FtcDashboard dashboard;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private MecanumDrive drive;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
//        TARGET_ID = 20;
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        velocityTopLut.add(-1, 0);
        velocityTopLut.add(0, 0);
        velocityTopLut.add(1, 0);
        velocityTopLut.add(46, 1400);
        velocityTopLut.add(53, 1300);
        velocityTopLut.add(70, 1270);
        velocityTopLut.add(118, 1400);
        velocityTopLut.add(140, 1400);
        velocityTopLut.createLUT();

        velocityBottomLut.add(-1, 0);
        velocityBottomLut.add(0, 0);
        velocityBottomLut.add(1, 0);
        velocityBottomLut.add(46, 1400);
        velocityBottomLut.add(53, 1300);
        velocityBottomLut.add(70, 1270);

        velocityBottomLut.add(118, 1400);
        velocityBottomLut.add(140, 1400);


        velocityBottomLut.createLUT();

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        // Vision portal
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();

        robot.init(hardwareMap);             // motors/servos/IMU setup// webcam + AprilTag setup
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        if (robotRange > 0 && autoPower) {
            Top_Target_Speed = velocityTopLut.get(robotRange);
            Bottom_Target_Speed = velocityBottomLut.get(robotRange);
        }

        robotRange = range;

        PIDFController topShooterController = new PIDFController(top_P, top_I, top_D, top_F);
        PIDFController bottomShooterController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);

        double topOutput = topShooterController.calculate(robot.TopMotor.getVelocity(), Top_Target_Speed);
        double bottomOutput = bottomShooterController.calculate(robot.BottomMotor.getVelocity(), Bottom_Target_Speed);

        double autoTurn = 0;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("TopMotorRPM", (robot.TopMotor.getVelocity() / 28.0) * 60.0);
        packet.put("BottomMotorRPM", (robot.BottomMotor.getVelocity() / 28.0) * 60.0);
        packet.put("TopCurrentA", robot.TopMotor.getCurrent(CurrentUnit.AMPS));
        packet.put("BottomCurrentA", robot.BottomMotor.getCurrent(CurrentUnit.AMPS));
        packet.put("BottomVelocity", robot.BottomMotor.getVelocity());
        packet.put("TopVelocity", robot.TopMotor.getVelocity());
        packet.put("TopTarget", Top_Target_Speed);
        packet.put("BottomTarget", Bottom_Target_Speed);
        dashboard.sendTelemetryPacket(packet);

//        if (gamepad1.left_bumper) shift = 0.3; // slow mode
//        if (gamepad1.right_bumper) shift = 1.0; // full speed

        if (((robot.TopMotor.getVelocity() == topOutput) && (bearing < 3 || bearing > -3) && (robot.BottomMotor.getVelocity() == bottomOutput))) {
            leftLightColor = 0.5;
            rightLightColor = 0.5;
        } else {
            leftLightColor = 0;
            rightLightColor = 0;
        }
        if (((robot.TopMotor.getVelocity() == topOutput) && (robot.BottomMotor.getVelocity() == bottomOutput))) {
            leftLightColor = 0.388;
            rightLightColor = 0.388;
        } else {
            leftLightColor = 0;
            rightLightColor = 0;
        }


        if (leftLightColor >= 0.001) {
            robot.leftLight.setPosition(leftLightColor);
        } else {
            robot.leftLight.setPosition(leftLightDeafualtColor);
        }
        if (rightLightColor >= 0.001) {
            robot.rightLight.setPosition(rightLightColor);
        } else {
            robot.rightLight.setPosition(rightLightDeafualtColor);
        }


        if (gamepad1.right_trigger > 0.1) {
            robot.intake.setVelocity(Intake_Speed);
        }
        if (gamepad1.y) {
            robot.intake.setVelocity(-Intake_Speed);
        } else {
            robot.intake.setPower(0.1);
        }



        if (Math.abs(gamepad2.right_trigger) > 0.1) {
            robot.BottomMotor.setVelocity(bottomOutput);
        } else {
            robot.BottomMotor.setPower(0.0);
        }

        if (Math.abs(gamepad2.left_trigger) > 0.1) {
            robot.TopMotor.setVelocity(topOutput);
        } else {
            robot.TopMotor.setPower(0.0);
        }
        if (gamepad2.dpadUpWasPressed()) {
            Top_Target_Speed += interval;
        }
        if (gamepad2.dpadDownWasPressed()) {
            Top_Target_Speed -= interval;
        }

        if (gamepad2.dpadRightWasPressed()) {
            Bottom_Target_Speed += interval;
        }
        if (gamepad2.dpadLeftWasPressed()) {
            Bottom_Target_Speed -= interval;
        }

        if (gamepad2.x && autoPower == false) {
            Top_Target_Speed = 1270;
            Bottom_Target_Speed = 1270;
        }

        if (gamepad2.b && autoPower == false) {
            Top_Target_Speed = 1400;
            Bottom_Target_Speed = 1400;

        }


        if (gamepad2.right_bumper || gamepad1.right_bumper) {
            robot.transfer.setPower(1);
        }
        if (gamepad2.left_bumper || gamepad2.a) {
            robot.transfer.setPower(-1);
        } else {
            robot.transfer.setPower(0);
        }

        if (gamepad2.psWasPressed() && autoPower == true) {
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
        if (gamepad1.dpadRightWasPressed() /*|| gamepad2.dpadRightWasPressed()*/) {
            TARGET_ID = 24;
            offset = -5;
        }
        if (gamepad1.dpadLeftWasPressed()/* || gamepad2.dpadLeftWasPressed()*/) {
            TARGET_ID = 20;
            offset = -3;
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


//        if (targetTag == null) {
//            range = targetTag.ftcPose.range;
//            bearing = targetTag.ftcPose.bearing;
//        }
        if (gamepad1.left_trigger > 0.9 && targetTag != null) {

            double bearing = targetTag.ftcPose.bearing;

            if (Math.abs(bearing) > aimTolorance) {    // 5-degree tolerance
                autoTurn = (bearing + offset) * rotationFactor;// proportional turn
            }
            telemetry.addData("range", targetTag.ftcPose.range);
////            drive.setDrivePowers(
////                    new PoseVelocity2d(
////                            new Vector2d(0, 0),   // no translation
////                            turnPower             // rotation
////                    )
////            );
        }
        if (gamepad1.left_trigger > 0.9 && targetTag == null) {
            gamepad1.runRumbleEffect(noEffect);
        }


        Pose2d pose = drive.localizer.getPose();// radians


//TODO: Combine joystick movement and camera auto centering movement


        drive.setDrivePowers(drive.localizer.getPose().heading.inverse().times(new PoseVelocity2d(
                new Vector2d(
                        (-gamepad1.left_stick_y * shift),
                        (-gamepad1.left_stick_x * shift)
                ),
                (-gamepad1.right_stick_x * shift) + autoTurn
        )));

        if (gamepad1.optionsWasPressed()) {
            drive.localizer.setPose(
                    new Pose2d(
                            drive.localizer.getPose().position,
                            0.0
                    )
            );
        }
//        else if(gamepad1.left_stick_x <=0 && gamepad1.left_stick_y <=0 && gamepad1.right_stick_x <=0) {
//                drive.setDrivePowers(
//                        new PoseVelocity2d(new Vector2d(0, 0), 0)
//                );
//        }

        // Update Road Runner’s localization
        drive.localizer.update();

        telemetry.addData("Intake Power", Intake_Speed);
        telemetry.addData("Top Shooter Target Speed", Top_Target_Speed);
        telemetry.addData("Bottom Shooter Target Speed", Bottom_Target_Speed);
        telemetry.addData("Ball Count", ballNumber);
        telemetry.addData("Selected Id", TARGET_ID);
        telemetry.addData("bearing", bearing);
        telemetry.addData("range", range);
        telemetry.addData("robotRange", robotRange);
        telemetry.addData("autopower", autoPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

}