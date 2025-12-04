package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@Autonomous(name = "Launch Auto Zone", group = "A")
public class AutoLaunchZone extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        boolean isRed = true;

        // ---------------- SELECT ALLIANCE ----------------
        telemetry.addLine("Press LEFT=RED  RIGHT=BLUE");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left)  isRed = true;
            if (gamepad1.dpad_right) isRed = false;

            telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
            telemetry.update();
        }

        // ---------------- CAMERA SETUP ----------------
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().build();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .build();

        // ---------------- STARTING POSE ----------------
        Pose2d startPose = isRed ?
                new Pose2d(0, 72, Math.toRadians(180)) :
                new Pose2d(0, -72, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- STEP 1: MOVE TO SHOOT POSITION ----------------
        double targetY = isRed ? 24 : -24;

        while (opModeIsActive() && Math.abs(drive.localizer.getPose().position.y - targetY) > 1.0) {
            double error = targetY - drive.localizer.getPose().position.y;
            double power = 0.2 * Math.signum(error);

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0, power),
                    0
            ));
            drive.localizer.update();
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        // ---------------- STEP 2: AUTO AIM ----------------
        double gain = 0.015;
        int targetId = isRed ? 24 : 20;

        while (opModeIsActive()) {

            List<AprilTagDetection> tags = tagProcessor.getDetections();
            AprilTagDetection tag = null;

            for (AprilTagDetection t : tags) {
                if (t.id == targetId) {
                    tag = t;
                    break;
                }
            }

            if (tag == null) {
                telemetry.addLine("Tag not found… turning slowly");
                telemetry.update();

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0.1
                ));
                sleep(50);
                continue;
            }

            double bearing = tag.ftcPose.bearing;

            if (Math.abs(bearing) < 1.5) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                telemetry.addLine("AIM LOCKED");
                telemetry.update();
                break;
            }

            double turn = bearing * gain;

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0, 0),
                    turn
            ));

            telemetry.addData("Bearing", bearing);
            telemetry.update();

            sleep(20);
        }

        // ---------------- STEP 3: SHOOT (PLACEHOLDER) ----------------
        telemetry.addLine("SHOOTING…");
        telemetry.update();
        sleep(1000);

        // ---------------- STEP 4: EXIT ----------------
        double exitX = isRed ? -12 : 12;

        while (opModeIsActive() && Math.abs(drive.localizer.getPose().position.x - exitX) > 1.0) {
            double error = exitX - drive.localizer.getPose().position.x;
            double power = 0.2 * Math.signum(error);

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(power, 0),
                    0
            ));
            drive.localizer.update();
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}