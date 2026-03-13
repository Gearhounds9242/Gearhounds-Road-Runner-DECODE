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
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Mechanisms.Vision;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Autonomous(name = "Blue_Far_NoMirror")
public class Blue_Far_NoMirror extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();

    int topVelocity = 1215;
    int bottomVelocity = 1215;
    int goalX = -70;
    int goalY = -60;
    MecanumDrive drive;

    // Starting pose — blue side (Y is positive, mirrored from red)
    Pose2d startPose = new Pose2d(new Vector2d(60, -14), Math.toRadians(180));

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
                drive.actionBuilder(startPose)
                        // Move to shoot position 1
                        .strafeToSplineHeading(new Vector2d(55, -14), Math.toRadians(194))
                        .stopAndAdd(new ParallelAction(drivetrain.turnTo(goalX,goalY)))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity, bottomVelocity),
                                        intake.runIntake(1, 1),
                                        new SequentialAction(
                                                new SleepAction(1.5),
                                                transfer.runTransfer(),
                                                new SleepAction(1),
                                                transfer.stopTransfer(),
                                                shooter.stopShooter()
                                        )
                                )
                        )
                        // Drive to first intake location
                        .strafeToSplineHeading(new Vector2d(35, -25), Math.toRadians(270))
                        .strafeToConstantHeading(new Vector2d(35, -60))
                        .stopAndAdd(transfer.tapTransfer())
                        // Return to shoot position 2
                        .strafeToSplineHeading(new Vector2d(55, -14), Math.toRadians(202))
                        .stopAndAdd(new ParallelAction(drivetrain.turnTo(goalX,goalY)))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity, bottomVelocity),
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
                        // Drive to second intake location
                        .strafeToSplineHeading(new Vector2d(12, -25), Math.toRadians(270))
                        .strafeToConstantHeading(new Vector2d(12, -60))
                        .stopAndAdd(transfer.tapTransfer())
                        // Return to shoot position 3
                        .strafeToSplineHeading(new Vector2d(55, -19), Math.toRadians(205))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity + 10, bottomVelocity + 10),
                                        intake.runIntake(1, 1),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                transfer.runTransfer(),
                                                new SleepAction(1),
                                                transfer.stopTransfer(),
                                                shooter.stopShooter()
                                        )
                                )
                        )
                        // Drive to third intake location
                        .strafeToSplineHeading(new Vector2d(60,-52.5), Math.toRadians(270))
                        .strafeTo(new Vector2d(65, -60))
                        .strafeTo(new Vector2d(65, -58))
                        .strafeTo(new Vector2d(65, -60))
                        .stopAndAdd(transfer.tapTransfer())
                        // Return to shoot position 4
                        .strafeToSplineHeading(new Vector2d(55, -14), Math.toRadians(198))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity + 20, bottomVelocity + 20),
                                        intake.runIntake(1, 1),
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
                        // Park and save pose
                        .strafeTo(new Vector2d(35, -14))
                        .stopAndAdd(new SavePose())
                        .build());
    }

    public class SavePose implements InstantFunction {
        @Override
        public void run() {
            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}