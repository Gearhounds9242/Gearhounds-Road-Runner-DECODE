package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Autonomous(name = "Red_Far")
public class Red_Far extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();

    int topVelocity = 1215;
    int bottomVelocity = 1215;
    MecanumDrive drive;
    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(60, 14), Math.toRadians(180));


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot (done ONCE)
        robot.init(hardwareMap);

        // Create drive AFTER hardwareMap is ready
        drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(robot);
        Intake intake = new Intake(robot);
        Transfer transfer = new Transfer(robot);


        waitForStart();


        if (isStopRequested()) return;
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(157))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity,bottomVelocity),
                                        intake.runIntake(1,1),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                transfer.runTransfer(),
                                                new SleepAction(1),
                                                transfer.stopTransfer(),
                                                shooter.stopShooter()
                                        )
                                )
                        )
                        .splineTo(new Vector2d(35, 25), Math.toRadians(90))
//                        .stopAndAdd(intake.runIntake(1, 0.1))
                        .strafeTo(new Vector2d(35, 60))
//                        .stopAndAdd(intake.stopIntake())
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(157))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity,bottomVelocity),
//                                        intake.runIntake(1,1),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                transfer.runTransfer(),
                                                new SleepAction(1),
                                                transfer.stopTransfer(),
                                                shooter.stopShooter()
                                        )
                                )
                        )
                        .splineTo(new Vector2d(12, 30), Math.toRadians(90))
                        .strafeTo(new Vector2d(12, 60))
//                        .stopAndAdd(intake.runIntake(1, 0.1))
//                        .stopAndAdd(intake.stopIntake())
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(157))
                        .waitSeconds(1)
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity, bottomVelocity),
                                        intake.runIntake(1,1),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                transfer.runTransfer(),
                                                new SleepAction(1),
                                                transfer.stopTransfer(),
                                                shooter.stopShooter()
                                        )
                                )
                        )
                        .splineToSplineHeading(new Pose2d(60, 52.5, Math.toRadians(90)), Math.toRadians(0))
//                        .stopAndAdd(intake.runIntake(1, 0.1))

                        .strafeTo(new Vector2d(60, 60))
                        .strafeTo(new Vector2d(60, 45))
//                        .stopAndAdd(intake.stopIntake())

                        .strafeTo(new Vector2d(52, 33))

                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(160))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity,bottomVelocity),
                                        intake.runIntake(1,1),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                transfer.runTransfer(),
                                                new SleepAction(1),
                                                transfer.stopTransfer(),
                                                shooter.stopShooter(),
                                                intake.stopIntake()
                                        )
                                )
                        )
                        .strafeTo(new Vector2d(35, 14))
                                .stopAndAdd(new SavePose())
                                .build());


    }


    /// Please don't go messing around in here if you don't know what you are doing proceed with CAUTION


    public class SavePose implements InstantFunction {
        @Override
        public void run() {
            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}
