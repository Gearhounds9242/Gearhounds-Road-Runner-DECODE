//package org.firstinspires.ftc.teamcode.Autonomous;
//
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantFunction;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Pose2dDual;
//import com.acmerobotics.roadrunner.PoseMap;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
//import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
//import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;
//import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
//import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
//
//@Autonomous(name = "Blue_Close_Gate")
//public class Blue_Close_Gate extends LinearOpMode {
//
//    private final GearhoundsHardware robot = new GearhoundsHardware();
//    int topVelocity = 1135;
//    int bottomVelocity = 1080;
//    MecanumDrive drive;
//    // Starting pose
//    Pose2d startPose = new Pose2d(new Vector2d(-50, -50), Math.toRadians(146.25));
//    PoseMap mirrorPoseMap = pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // Initialize robot (done ONCE)
//        robot.init(hardwareMap);
//
//        // Create drive AFTER hardwareMap is ready
//        drive = new MecanumDrive(hardwareMap, startPose);
//        Shooter shooter = new Shooter(robot);
//        Intake intake = new Intake(robot);
//        Transfer transfer = new Transfer(robot);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//        Action path = drive.actionBuilder(startPose)
//
//                ///go to shoot location first time
//                .splineToConstantHeading(new Vector2d(-49, -40), Math.toRadians(58.5))
//                .splineToSplineHeading(new Pose2d(-18, -10, Math.toRadians(223.5)), Math.toRadians(60))
//                .waitSeconds(0.05)
//                //shoot preload3
//                .stopAndAdd(
//                        new SequentialAction(
//                                new ParallelAction(
//                                        intake.runIntake(1,1),
//                                        shooter.shootSequence(topVelocity, bottomVelocity, 3, 1, 1)
//                                ),
//                                transfer.runTransfer(),
//                                new SleepAction(1),
//                                transfer.stopTransfer(),
//                                shooter.stopShooter()
//                        )
//                )
//                ///go to 1st spike mark
////                .splineToSplineHeading(new Pose2d(-11.5, -29.5, Math.toRadians(270)), Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-11.5,-29.5, Math.toRadians(270)), Math.toRadians(-90))
//                //turn on intake
////                .stopAndAdd(intake.runIntake(1, 0.1))
//                .waitSeconds(0.05)
//                .splineToConstantHeading(new Vector2d(-11.5, -53.5), Math.toRadians(-90))
//                //tap transfer
//                .stopAndAdd(transfer.tapTransfer())
//                .waitSeconds(0.05)
//                .strafeTo(new Vector2d(-14,-40))
//                //intake off
//                .stopAndAdd(intake.stopIntake())
//                /// hit gate
////                .splineToSplineHeading(new Pose2d(-9, -34, Math.toRadians(90)), Math.toRadians(-90))
////                .splineToConstantHeading(new Vector2d(0, -34), Math.toRadians(-90))
//                .strafeTo(new Vector2d(-14,-33))
////                ///go to shoot location second time
//                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(223.5)), Math.toRadians(270))
//                //shoot 3
//                .stopAndAdd(
//                        new SequentialAction(
//                                new ParallelAction(
//                                        intake.runIntake(1,1),
//                                        shooter.shootSequence(topVelocity, bottomVelocity, 3, 1, 1)
//                                ),
//                                transfer.runTransfer(),
//                                new SleepAction(1),
//                                transfer.stopTransfer(),
//                                shooter.stopShooter()
//                        )
//                )
//                .waitSeconds(1)
//                ///go to 2nd spike mark
//                .strafeToLinearHeading(new Vector2d(12,-20), Math.toRadians(280))
////                .splineToSplineHeading(new Pose2d(12, -29.5, Math.toRadians(270)), Math.toRadians(10))
//                //turn on intake
//                .stopAndAdd(intake.runIntake(1, 0.1))
//                .waitSeconds(0.05)
//                .strafeTo(new Vector2d(13,-53.5))
//
////                .splineToConstantHeading(new Vector2d(12, -53.5), Math.toRadians(-90))
//                //tap transfer
//                .stopAndAdd(transfer.tapTransfer())
//                .waitSeconds(0.05)
//                .splineToConstantHeading(new Vector2d(12, -49), Math.toRadians(-90))
//                //intake off
//                ///go to shoot location third time
//                .splineToConstantHeading(new Vector2d(12, -20), Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(223.5)), Math.toRadians(10))
//                .waitSeconds(0.05)
//                //shoot 3
//                .stopAndAdd(
//                        new SequentialAction(
//                                new ParallelAction(
//                                        intake.runIntake(1,1),
//                                        shooter.shootSequence(topVelocity, bottomVelocity, 3, 1, 1)
//                                ),
//                                transfer.runTransfer(),
//                                new SleepAction(1),
//                                transfer.stopTransfer(),
//                                shooter.stopShooter()
//                        )
//                )
//                ///go out of the launch zone
//                .splineToSplineHeading(new Pose2d(-23.5, -50, Math.toRadians(270)), Math.toRadians(270))
//                /// save pos for teleop
//                .stopAndAdd(new SavePose())
//                .build();
//        Actions.runBlocking(new SequentialAction(path));
//
//
//    }
//
//
//    /// Please don't go messing around in here if you don't know what you are doing proceed with CAUTION
//
//
//    public class SavePose implements InstantFunction {
//        @Override
//        public void run() {
//            PoseStorage.currentPose = drive.localizer.getPose();
//        }
//    }
//}
