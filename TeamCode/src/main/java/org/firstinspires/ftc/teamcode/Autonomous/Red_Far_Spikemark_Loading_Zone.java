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

@Autonomous(name = "Red_Far_Spikemark_Loading_Zone")
public class Red_Far_Spikemark_Loading_Zone extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();

    int topVelocity = 1230;
    int bottomVelocity = 1230;
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
        Vision vision = new Vision(robot);
        vision.setDrive(drive);
        Drivetrain drivetrain = new Drivetrain(drive);

        waitForStart();


        if (isStopRequested()) return;
        Actions.runBlocking(
                drive.actionBuilder(startPose)

                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(151))
                        // shoot
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


                        //pick up
                        .strafeToSplineHeading(new Vector2d(35, 25), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(35, 60))
                        .stopAndAdd(transfer.tapTransfer())


                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(152))
                        //shoot
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity, bottomVelocity),
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

                        // intake
                        .strafeToLinearHeading(new Vector2d(60, 14), Math.toRadians(90))
                        .strafeTo(new Vector2d(60, 60))
                        .stopAndAdd(transfer.tapTransfer())
                        .strafeTo(new Vector2d(60, 50))
                        .strafeTo(new Vector2d(60, 60))

                        //Shoot
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(156))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity, bottomVelocity),
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


                        // intake
                        .strafeToSplineHeading(new Vector2d(35, 25), Math.toRadians(90))
                        .strafeTo(new Vector2d(35, 60))
                        .stopAndAdd(transfer.tapTransfer())
                        .strafeTo(new Vector2d(35, 50))
                        .strafeTo(new Vector2d(35, 60))

                        //Shoot
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(153))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity, bottomVelocity),
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


                        // intake
                        .strafeToLinearHeading(new Vector2d(60, 14), Math.toRadians(90))
                        .strafeTo(new Vector2d(60, 60))
                        .stopAndAdd(transfer.tapTransfer())
                        .strafeTo(new Vector2d(60, 50))
                        .strafeTo(new Vector2d(60, 60))


                        //Shoot
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(153))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity, bottomVelocity),
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

                        .strafeToSplineHeading(new Vector2d(60, 60), Math.toRadians(90))
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
