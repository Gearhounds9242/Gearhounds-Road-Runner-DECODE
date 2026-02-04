package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Autonomous(name = "Red_Close")
public class Red_Close extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();
    MecanumDrive drive;
    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(-50, 50), Math.toRadians(-146.25));


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot (done ONCE)
        robot.init(hardwareMap);

        // Create drive AFTER hardwareMap is ready
        drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(robot);
        Intake intake = new Intake(robot);

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(-11.6, 11.6, Math.toRadians(139)), Math.toRadians(-4))
                .waitSeconds(3)
                //    shoot
                .splineToSplineHeading(new Pose2d(-12,30, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-12, 52.5))
                        .stopAndAdd(intake.runIntake(1,5))
                .strafeTo(new Vector2d(-12, 50))
                .splineToLinearHeading(new Pose2d(-12, 52.5, Math.toRadians(90)), Math.toRadians(-11))
                .splineToSplineHeading(new Pose2d(-11.6, 11.6, Math.toRadians(139)), Math.toRadians(1))
                // shoot
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(0))
                .strafeTo(new Vector2d(12, 60))
                        .stopAndAdd(intake.runIntake(1,5))
                .strafeTo(new Vector2d(12, 45))
                .splineToSplineHeading(new Pose2d(-11.6, 11.6, Math.toRadians(139)), Math.toRadians(1))
                // shoot
                .waitSeconds(1)
                .strafeTo(new Vector2d(12, 20))

                        .build());


    }


    /// Please don't go messing around in here if you don't know what you are doing proceed with CAUTION


    public class SavePose implements InstantFunction {
        @Override
        public void run() {PoseStorage.currentPose = drive.localizer.getPose();}
    }
}
