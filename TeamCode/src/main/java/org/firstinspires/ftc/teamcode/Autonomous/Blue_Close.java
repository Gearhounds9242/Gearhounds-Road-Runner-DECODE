package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.Action;
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

@Autonomous(name = "Blue_Close")
public class Blue_Close extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();
    int topVelocity = 1125;
    int bottomVelocity = 1200;
    MecanumDrive drive;
    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(-50, -50), Math.toRadians(146.25));
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
        Action path = drive.actionBuilder(startPose)
///go to shoot location first time
//                .splineToConstantHeading(new Vector2d(-49, -49), Math.toRadians(58.5))
                .strafeToLinearHeading(new Vector2d(-15.8,-5.8), Math.toRadians(221.5))
                .stopAndAdd(drivetrain.turnTo(-63.2,-55.9))
//

                ///shoot preload3
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
                ///go to 1st spike mark
                .strafeToSplineHeading(new Vector2d(-11.5,-29.5), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-11.5, -53.5), Math.toRadians(-90))
//                //tap transfer
                .stopAndAdd(transfer.tapTransfer())
                .strafeToLinearHeading(new Vector2d(-13.8,-5.8), Math.toRadians(220))
                .stopAndAdd(drivetrain.turnTo(-63.2,-55.9))

//                .waitSeconds(0.5)

                ///shoot 3
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


                .strafeToSplineHeading(new Vector2d(15, -22), Math.toRadians(280))
                .strafeToConstantHeading(new Vector2d(15,-60))
                .strafeToConstantHeading(new Vector2d(15,-29))
                .strafeToLinearHeading(new Vector2d(-15.8,-5.8), Math.toRadians(220))

//                .waitSeconds(0.5)
                .stopAndAdd(drivetrain.turnTo(-63.2,-55.9))

///                shoot 3
                .stopAndAdd(
                        new ParallelAction(
                                shooter.runShooter(topVelocity,bottomVelocity+100),
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


                .splineToSplineHeading(new Pose2d(-23.5, -50, Math.toRadians(270)), Math.toRadians(270))

//                /// save pos for teleop
                .stopAndAdd(shooter.stopShooter())
                .stopAndAdd(new SavePose())
                .build();
        Actions.runBlocking(new SequentialAction(path));


    }


    /// Please don't go messing around in here if you don't know what you are doing proceed with CAUTION


    public class SavePose implements InstantFunction {
        @Override
        public void run() {
            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}
