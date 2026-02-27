package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.BilinearInterpolator;
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
    public static double rotationFactor = -0.03;
    public static double aimTolorance = 0.2;
    public static int ballNumber = 0;
    public static double offset = 0;
    public static double top_P = 0.0018;
    public static double top_I = 0;
    public static double top_D = 0;
    public static double top_F = 0.0004;
    public static double bottom_P = 0.0018;
    public static double bottom_I = 0;
    public static double bottom_D = 0;
    public static double bottom_F = 0.0004;
    public static double rightLightColor = 0;
    public static double leftLightColor = 0;
    public static double leftLightDeafualtColor = 0;
    public static double rightLightDeafualtColor = 0;
    public static double range;
    public static double robotRange;
    public static boolean autoPower = true;
    public static double farOffset = 0;
    public static double closeOffset = 0;
    public static double bearing = 0;
    public static double shooterTolerance;
    public static double transfer_velocity = 2000;
    static double ROLLER_VELOCITY_TOLERANCE = 65;
    private double headingOffset;
    private final GearhoundsHardware robot = new GearhoundsHardware();
    private final ElapsedTime runtime = new ElapsedTime();
    public boolean canSeeTag;
    InterpLUT velocityTopLut = new InterpLUT();
    InterpLUT velocityBottomLut = new InterpLUT();
    PIDFController topShooterController;
    PIDFController bottomShooterController;
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
    private MecanumDrive drive;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 180, 0, 0);

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
//        TARGET_ID = 20;
        telemetry.addData("Status", "Initializing...");
        telemetry.update();


        BilinearInterpolator cameraOffset = new BilinearInterpolator()
                .add(0,  0,  0.30)
                .add(0,  45, 0.45)
                .add(0,  90, 0.60)
                .add(12, 0,  0.50)
                .add(12, 45, 0.65)
                .add(12, 90, 0.80)
                .add(24, 0,  0.70)
                .add(24, 45, 0.85)
                .add(24, 90, 1.00);


        velocityTopLut.add(-1, 0);
        velocityTopLut.add(0, 0);
        velocityTopLut.add(66,1125);
        velocityTopLut.add(86,1125);
        velocityTopLut.add(125,1170);
        velocityTopLut.add(134.5,1241);
        velocityTopLut.add(190, 1400);
        velocityTopLut.createLUT();
//
        velocityBottomLut.add(-1, 0);
        velocityBottomLut.add(0, 0);
        velocityBottomLut.add(66,1200);
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

        robot.init(hardwareMap);             // motors/servos/IMU setup// webcam + AprilTag setup
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        topShooterController = new PIDFController(top_P, top_I, top_D, top_F);
        bottomShooterController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);
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


        topShooterController.setPIDF(top_P, top_I, top_D, top_F);
        bottomShooterController.setPIDF(bottom_P, bottom_I, bottom_D, bottom_F);


        double topOutput = topShooterController.calculate(robot.topMotor.getVelocity(), Top_Target_Speed);
        double bottomOutput = bottomShooterController.calculate(robot.bottomMotor.getVelocity(), Bottom_Target_Speed);
        boolean topReady = Math.abs(Top_Target_Speed - robot.topMotor.getVelocity()) < ROLLER_VELOCITY_TOLERANCE;
        boolean bottomReady = Math.abs(Bottom_Target_Speed - robot.bottomMotor.getVelocity()) < ROLLER_VELOCITY_TOLERANCE;

        double autoTurn = 0;

        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("TopMotorRPM", (robot.TopMotor.getVelocity() / 28.0) * 60.0);
//        packet.put("BottomMotorRPM", (robot.BottomMotor.getVelocity() / 28.0) * 60.0);
//        packet.put("TopCurrentA", robot.TopMotor.getCurrent(CurrentUnit.AMPS));
//        packet.put("BottomCurrentA", robot.BottomMotor.getCurrent(CurrentUnit.AMPS));
//        packet.put("IntakeRPM", (robot.intake.getVelocity() / 28) * 60);
        packet.put("BottomVelocity", robot.bottomMotor.getVelocity());
        packet.put("TopVelocity", robot.topMotor.getVelocity());
        packet.put("TopTarget", Top_Target_Speed);
        packet.put("BottomTarget", Bottom_Target_Speed);
        dashboard.sendTelemetryPacket(packet);

//        if (gamepad1.left_bumper) shift = 0.3; // slow mode
//        if (gamepad1.right_bumper) shift = 1.0; // full speed

//        if (((robot.TopMotor.getVelocity() == topOutput) && (bearing < 3 || bearing > -3) && (robot.BottomMotor.getVelocity() == bottomOutput))) {
//            leftLightColor = 0.5;
//            rightLightColor = 0.5;
//        } else {
//            leftLightColor = 0;
//            rightLightColor = 0;
//        }
//        if (((robot.TopMotor.getVelocity() == topOutput) && (robot.BottomMotor.getVelocity() == bottomOutput))) {
//            leftLightColor = 0.388;
//            rightLightColor = 0.388;
//        } else {
//            leftLightColor = 0;
//            rightLightColor = 0;
//        }


        if (gamepad1.ps){
            drive.localizer.setPose(new Pose2d(60.5, 61, 180));
        }

        if (topReady == true && bottomReady == true) {
            leftLightColor = 0.583;
            rightLightColor = 0.583;
        } else {
            leftLightColor = 0;
            rightLightColor = 0;
        }

        if ((Math.abs(bearing) < 3) && bearing >= 0.1) {
            leftLightColor = 0.388;
            rightLightColor = 0.388;
        } else {
            leftLightColor = 0;
            rightLightColor = 0;
        }

        if ((topReady && bottomReady) && (Math.abs(bearing) < 3)) {
            leftLightColor = 0.472;
            rightLightColor = 0.472;
        } else {
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



        if (gamepad1.dpadUpWasPressed()){
            offset += 1;
        }
        if (gamepad1.dpadDownWasPressed()){
            offset -= 1;
        }


        if (gamepad1.right_trigger > 0.1) {
            robot.intake.setPower(1);
        }
        else {
            robot.intake.setPower(0);
        }
        if (gamepad1.y) {
            robot.intake.setPower(-1);
        }


        if (Math.abs(gamepad2.right_trigger) > 0.1) {
            robot.bottomMotor.setPower(bottomOutput);
        } else {
            robot.bottomMotor.setPower(0.0);
        }

        if (Math.abs(gamepad2.left_trigger) > 0.1) {
            robot.topMotor.setPower(topOutput);
        } else {
            robot.topMotor.setPower(0.0);
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

        if (gamepad2.x && !autoPower) {
            Top_Target_Speed = 1270;
            Bottom_Target_Speed = 1270;
        }

        if (gamepad2.b && !autoPower) {
            Top_Target_Speed = 1400;
            Bottom_Target_Speed = 1400;

        }


//        if(gamepad2.b && autoPower == true){
//            offset = farOffset;
//        }
//        if(gamepad2.b && autoPower == true){
//            offset = closeOffset;
//
//        }
        if (gamepad2.right_bumper) {
            robot.transfer.setVelocity(transfer_velocity);
            robot.intake.setPower(1);
        }
        if (gamepad1.right_bumper) {
            robot.transfer.setVelocity(transfer_velocity);

        }
        if (gamepad2.left_bumper || gamepad2.a) {
            robot.transfer.setPower(-transfer_velocity);
        } else {
            robot.transfer.setPower(0);
        }

        if (gamepad2.psWasPressed() && autoPower == true) {
            autoPower = false;
            leftLightDeafualtColor = 0.333;
            rightLightDeafualtColor = 0.333;
        }
        if (gamepad2.shareWasPressed() && autoPower == false) {
            autoPower = true;
            leftLightDeafualtColor = 0;
            rightLightDeafualtColor = 0;
        }



/*
CAMERA STUFF
 */
        if (gamepad1.dpadRightWasPressed() /*|| gamepad2.dpadRightWasPressed()*/) {
            TARGET_ID = 24;
            offset = 2.1;
        }
        if (gamepad1.dpadLeftWasPressed()/* || gamepad2.dpadLeftWasPressed()*/) {
            TARGET_ID = 20;
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



//TODO: Combine joystick movement and camera auto centering movement


        // Options button - reset driver's forward reference
        if (gamepad1.optionsWasPressed()) {
            // Store current robot heading as the new "forward" reference
            // without touching the localizer pose at all
            headingOffset = drive.localizer.getPose().heading.toDouble();
        }

// Then in the drive.setDrivePowers call, apply the offset:
        drive.setDrivePowers(
                Rotation2d.exp(-drive.localizer.getPose().heading.toDouble() + headingOffset)
                        .times(new PoseVelocity2d(
                                new Vector2d(
                                        (-gamepad1.left_stick_y * shift),
                                        (-gamepad1.left_stick_x * shift)
                                ),
                                (-gamepad1.right_stick_x * shift) + autoTurn
                        ))
        );
//        else if(gamepad1.left_stick_x <=0 && gamepad1.left_stick_y <=0 && gamepad1.right_stick_x <=0) {
//                drive.setDrivePowers(
//                        new PoseVelocity2d(new Vector2d(0, 0), 0)
//                );
//        }

        // Update Road Runner’s localization
        drive.localizer.update();

//        telemetry.addData("Intake Power", Intake_Speed);
//        telemetry.addData("Top Shooter Target Speed", Top_Target_Speed);
//        telemetry.addData("Bottom Shooter Target Speed", Bottom_Target_Speed);
//        telemetry.addData("Ball Count", ballNumber);
        telemetry.addData("Selected Id", TARGET_ID);
//        telemetry.addData("bearing", bearing);
        telemetry.addData("range", range);
//        telemetry.addData("robotRange", robotRange);
        telemetry.addData("autopower", autoPower);
        telemetry.addData("X",drive.localizer.getPose().position.x);
        telemetry.addData("Y",drive.localizer.getPose().position.y);
        telemetry.addData("Heading",drive.localizer.getPose().heading);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

}