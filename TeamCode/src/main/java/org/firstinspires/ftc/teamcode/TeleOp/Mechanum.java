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
import com.sun.tools.javac.comp.Todo;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveToAprilTagOmni;
import org.firstinspires.ftc.teamcode.MecanumDrive;

//import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.Utilities.VisionSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.seattlesolvers.solverslib.controller.PController;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.Utilities.Leds;

import java.util.List;


/**
 * Main TeleOp for field-centric mecanum drive + intake + shooter + drop.
 * Also includes AprilTag auto-alignment:
 *   - Only adjusts lateral (strafe) and heading (yaw)
 *   - Does NOT change forward/back distance
 *   - Only aligns to your own goal (filter by tag ID + alliance flag)
 */
@Config
@TeleOp(name = "Mechanum", group = "TeamCode/TeleOp")
public class Mechanum extends OpMode {
    public static int  TARGET_ID = 0 ;
    private final GearhoundsHardware robot = new GearhoundsHardware();
    private final ElapsedTime runtime = new ElapsedTime();
//     InterpLUT velocityTopLut = new InterpLUT();
//     InterpLUT velocityBottomLut = new InterpLUT();
    private FtcDashboard dashboard;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private Leds leds;

    public static boolean isRedAlliance = false;   // set false when you are blue
    private MecanumDrive drive;
    public static double Intake_Speed = 1000.0;
    // back power 1650 for both
    // front power 1600 bottom 1480 top
    public static double Top_Speed = 2000.0;
    public static double Bottom_Speed = 2000.0;
    public static double shift = 1.0;
    // Drop servo + ball count logic
    public static double drop_up = 0.63;
    public static double drop_down = 0.36;
    public static double drop_high = 0.36;
    public static double rotationFactor = 0.05;
    public static double aimTolorance = 1;

    public static double p2ytime = 0.0;
    public static int ballNumber = 0;
    public static double offset = 0;

//        PIDController motor = new PIDController(kP, kI, kD);

    // two-ball timing (seconds)
    public static double twoballtime1 = 0.1;
    public static double twoballtime2 = 0.5;
    public static double twoballtime3 = 1.0;

    // one-ball timing (seconds)
    public static double oneballtime1 = 0.1;
    public static double oneballtime2 = 0.5;

    public static double time1 = 1;
    public static double time2 = 2;



    public static double blocktime1 = 0.1;
    public static double blocktime2 = 0.4;


    public static double blockpos1 = 0.45;
    public static double blockpos2 = 0.2;
    private static double interval = 10;

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


    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

//         velocityTopLut.add(72, 1480);
//         velocityTopLut.add(48,1180);
//         velocityTopLut.add(108.5,1650);
//         velocityTopLut.add(96, 1550);
//         velocityTopLut.createLUT();
//
//         velocityBottomLut.add(72, 1540);
//         velocityBottomLut.add(48,2000);
//         velocityBottomLut.add(108.5,1650);
//         velocityBottomLut.add(96, 1550);
//         velocityBottomLut.createLUT();
        
//        tagProcessor = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .build();
//
//        // Vision portal
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(tagProcessor)
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCameraResolution(new Size(640, 480))
//                .enableLiveView(true)
//                .build();

        robot.init(hardwareMap);             // motors/servos/IMU setup// webcam + AprilTag setup
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }
    
    @Override
    public void start() {
        runtime.reset();
        if(!gamepad1.a || !gamepad2.a){
        robot.block.setPosition(0.45);}

//        robot.drop.setPosition(0.63);
    }


    @Override
    public void loop() {

        double autoTurn = 0;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("TopMotorRPM", (robot.TopMotor.getVelocity() / 28.0) * 60.0);
        packet.put("BottomMotorRPM", (robot.BottomMotor.getVelocity() / 28.0) * 60.0);
        packet.put("TopCurrentA", robot.TopMotor.getCurrent(CurrentUnit.AMPS));
        packet.put("BottomCurrentA", robot.BottomMotor.getCurrent(CurrentUnit.AMPS));
        dashboard.sendTelemetryPacket(packet);

        if (gamepad1.left_bumper) shift = 0.3; // slow mode
        if (gamepad1.right_bumper) shift = 1.0; // full speed

        if (gamepad1.right_trigger > 0.1) {
            robot.intake.setVelocity(Intake_Speed);
        } else {
            robot.intake.setPower(0.0);
        }
        if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
            TARGET_ID = 24;
            offset = -5;
        }
        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
            TARGET_ID = 20;
            offset = -3;
        }

        if (gamepad2.right_trigger > 0.1) {
            robot.BottomMotor.setVelocity(Bottom_Speed * gamepad2.right_trigger);
        } else {
            robot.BottomMotor.setPower(0.0);
        }

        if (gamepad2.left_trigger > 0.1) {
            robot.TopMotor.setVelocity(Top_Speed * gamepad2.left_trigger);
        } else {
            robot.TopMotor.setPower(0.0);
        }

        if (gamepad2.y) {
            p2ytime = runtime.seconds();
        }


        if(gamepad2.x){
            Top_Speed = 1480;
            Bottom_Speed = 1540;
        }

        if(gamepad2.b){
            Top_Speed = 1650;
            Bottom_Speed = 1650;

        }


        if(gamepad2.right_bumper){
            robot.transfer.setVelocity(1000);
        }
        else {
            robot.transfer.setPower(0);
        }


        if ((runtime.seconds() - p2ytime) < blocktime1) {
            robot.block.setPosition(blockpos1);
        } else if ((runtime.seconds() - p2ytime) < blocktime2) {
            robot.block.setPosition(blockpos2);
        }

        if (gamepad2.ps){
            robot.block.setPosition(0.45);
        }


        if (gamepad2.dpadUpWasPressed()){
            Top_Speed += interval;
        }
        if (gamepad2.dpadDownWasPressed()){
            Top_Speed -= interval;
        }

        if (gamepad2.dpadRightWasPressed()){
            Bottom_Speed += interval;
        }
        if (gamepad2.dpadLeftWasPressed()){
            Bottom_Speed -= interval;
        }


        //        // Two-ball sequence
//        if (((runtime.seconds() - p2ytime) < twoballtime1) && ballNumber >= 2) {
//            robot.drop.setPosition(0.28);
//        } else if (((runtime.seconds() - p2ytime) < twoballtime2) && ballNumber >= 2) {
//            robot.drop.setPosition(0.69);
//        } else if (((runtime.seconds() - p2ytime) < twoballtime3) && ballNumber >= 2) {
//            robot.drop.setPosition(0.63);
//            ballNumber -= 1;
//        }
//
//        // One-ball sequence
//        if (((runtime.seconds() - p2ytime) < oneballtime1) && ballNumber <= 1) {
//            robot.drop.setPosition(0.28);
//        } else if (((runtime.seconds() - p2ytime) < oneballtime2) && ballNumber <= 1) {
//            robot.drop.setPosition(0.63);
//            ballNumber = 0;
//        }

        // Manual ball count adjustments
//        if (gamepad2.dpadDownWasPressed()){
//            ballNumber--;
//            gamepad2.rumble(450,0,100);
//        }
//        if (gamepad2.dpadUpWasPressed()){
//            ballNumber++;
//            gamepad2.rumble(0,10000,150);
//        }

//        // Manual drop override
//        if (gamepad2.dpad_right) {
//            robot.drop.setPosition(drop_high);
//        }



/*
CAMERA STUFF
 */
        // Get detections
//        List<AprilTagDetection> detections = tagProcessor.getDetections();
//
//        // Find ONLY the target ID
//        AprilTagDetection targetTag = null;
//        for (AprilTagDetection goalTag : detections) {
//            if (goalTag.id == TARGET_ID) {
//                targetTag = goalTag;
//                break;
//            }
//        }
//        if (gamepad1.right_trigger > 0.9 && targetTag != null) {
//
//            double bearing = targetTag.ftcPose.bearing;
//
//            if (Math.abs(bearing) > aimTolorance) {    // 5-degree tolerance
//                autoTurn = (bearing + offset) * rotationFactor;// proportional turn
//            }
//            telemetry.addData("range",targetTag.ftcPose.range);
////            drive.setDrivePowers(
////                    new PoseVelocity2d(
////                            new Vector2d(0, 0),   // no translation
////                            turnPower             // rotation
////                    )
////            );
//       }
//            if (gamepad1.right_trigger > 0.9 && targetTag == null) {
//                gamepad1.runRumbleEffect(noEffect);
//            }


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
            telemetry.addData("Top Shooter Scale", Top_Speed);
            telemetry.addData("Bottom Shooter Scale", Bottom_Speed);
            telemetry.addData("Ball Count", ballNumber);
            telemetry.addData("Selected Id", TARGET_ID);
            telemetry.update();
        }
    }



