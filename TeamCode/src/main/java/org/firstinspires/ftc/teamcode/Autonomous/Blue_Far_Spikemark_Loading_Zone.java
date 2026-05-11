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

@Autonomous(name = "Blue_Far_Spikemark_Loading_Zone")
public class Blue_Far_Spikemark_Loading_Zone extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();

    int topVelocity = 1230;
    int bottomVelocity = 1230;
    int goalX = -70;
    int goalY = -60;
    MecanumDrive drive;
    // Starting pose
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
                                        shooter.runShooter(topVelocity-10, bottomVelocity-10),
                                        intake.runIntake(1, 0.1),
                                        new SequentialAction(
                                                new SleepAction(1),
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


                        // Move to shoot position 2
                        .strafeToSplineHeading(new Vector2d(55, -14), Math.toRadians(196.8))
                        .stopAndAdd(new ParallelAction(drivetrain.turnTo(goalX,goalY)))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity-10, bottomVelocity-10),
                                        intake.runIntake(1, 0.1),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                transfer.runTransfer(),
                                                new SleepAction(1),
                                                transfer.stopTransfer(),
                                                shooter.stopShooter()
                                        )
                                )
                        )

                        // Drive to intake position 2
                        .strafeToSplineHeading(new Vector2d(65,-52.5), Math.toRadians(270))
                        .strafeTo(new Vector2d(65, -60))
                        .strafeTo(new Vector2d(65, -55))
                        .strafeTo(new Vector2d(65, -60))
                        .stopAndAdd(transfer.tapTransfer())
                        // Move to shoot position 3
                        .strafeToSplineHeading(new Vector2d(55, -14), Math.toRadians(194))
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

                        // Drive to intake position 3
                        .strafeToSplineHeading(new Vector2d(35,-48), Math.toRadians(270))
                        .strafeTo(new Vector2d(35, -60))
                        .strafeTo(new Vector2d(35, -58))
                        .strafeTo(new Vector2d(35, -60))
                        .stopAndAdd(transfer.tapTransfer())
                        // Move to shoot position 4
                        .strafeToSplineHeading(new Vector2d(55, -14), Math.toRadians(196.7))
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


                        // Drive to intake position 4
                        .strafeToSplineHeading(new Vector2d(59,-48), Math.toRadians(270))
                        .strafeTo(new Vector2d(59, -60))
                        .strafeTo(new Vector2d(59, -58))
                        .strafeTo(new Vector2d(59, -60))

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
