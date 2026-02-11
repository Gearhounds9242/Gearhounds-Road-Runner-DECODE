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
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Autonomous(name = "Blue_Close")
public class Blue_Close extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();
    int topVelocity = 1135;
    int bottomVelocity = 1080;
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

        waitForStart();

        if (isStopRequested()) return;
        Action path = drive.actionBuilder(startPose)
///go to shoot location first time
//                .splineToConstantHeading(new Vector2d(-49, -49), Math.toRadians(58.5))
                .strafeToLinearHeading(new Vector2d(-20,-12), Math.toRadians(223.5))

//                .waitSeconds(0.5)

                ///shoot preload3
                .stopAndAdd(
                        new ParallelAction(
                                shooter.runShooter(1100,1100),
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
                .strafeToSplineHeading(new Vector2d(-20,-12), Math.toRadians(223.5))

//                .waitSeconds(0.5)

                ///shoot 3
                .stopAndAdd(
                        new ParallelAction(
                                shooter.runShooter(1100,1100),
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


                .strafeToSplineHeading(new Vector2d(13.5, -26), Math.toRadians(275))
                .strafeToConstantHeading(new Vector2d(13.5,-60))
                .strafeToConstantHeading(new Vector2d(13.5,-29))
                .strafeToSplineHeading(new Vector2d(-20,-12), Math.toRadians(223.5))

//                .waitSeconds(0.5)

//                shoot 3
                .stopAndAdd(
                        new ParallelAction(
                                shooter.runShooter(1100,1100),
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
