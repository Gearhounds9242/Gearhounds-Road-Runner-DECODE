package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Mechanisms.Vision;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Autonomous(name = "Blue_Far")
public class Blue_Far extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();

    int topVelocity = 1186;
    int bottomVelocity = 1186;
    MecanumDrive drive;
    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(60, -14), Math.toRadians(180));
    PoseMap mirrorPoseMap = pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot (done ONCE)
        robot.init(hardwareMap);

        // Create drive AFTER hardwareMap is ready
        drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(robot);
        Intake intake = new Intake(robot);
        Transfer transfer = new Transfer(robot);
        Vision vision = new Vision(robot);
        vision.setDrive(drive);
        Drivetrain drivetrain = new Drivetrain(drive);

        waitForStart();


        if (isStopRequested()) return;
        Actions.runBlocking(
                drive.actionBuilder(startPose, mirrorPoseMap)
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(158))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity,bottomVelocity),
                                        intake.runIntake(1,1),
                                        new SequentialAction(
                                                new SleepAction(1.5),
                                                transfer.runTransfer(),
                                                new SleepAction(1),
                                                transfer.stopTransfer(),
                                                shooter.stopShooter()
                                        )
                                )
                        )
                        .strafeToSplineHeading(new Vector2d(35,25), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(35, 60))
                        .stopAndAdd(transfer.tapTransfer())
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(158))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity,bottomVelocity),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                transfer.runTransfer(),
                                                new SleepAction(1),
                                                transfer.stopTransfer(),
                                                shooter.stopShooter()
                                        )
                                )
                        )
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(12, 25), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(12, 60))
                        .stopAndAdd(transfer.tapTransfer())
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(158))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity+10, bottomVelocity+10),
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
                        .strafeTo(new Vector2d(60, 60))
                        .stopAndAdd(transfer.tapTransfer())
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(158))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity+10,bottomVelocity+10),
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
