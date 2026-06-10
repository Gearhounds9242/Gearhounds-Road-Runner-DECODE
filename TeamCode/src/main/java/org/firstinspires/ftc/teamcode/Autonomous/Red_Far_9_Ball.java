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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Mechanisms.Vision;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "Red_Far_9_Ball")
public class Red_Far_9_Ball extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();

    public int topVelocity = 1230;
    public int bottomVelocity = 1230;
    private VisionPortal visionPortal;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private WhiteBalanceControl whiteBalanceControl;
    private PtzControl ptzControl = null;
    public static long EXPOSURE_MS = 6;
    public static int GAIN = 0;
    public static int WHITE_BALANCE_K = 4000;
    MecanumDrive drive;
    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(60, 14), Math.toRadians(180));

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
//        exposureControl  = visionPortal.getCameraControl(ExposureControl.class);
//        gainControl      = visionPortal.getCameraControl(GainControl.class);
//
//        // Switch to manual mode so our values are respected
//        exposureControl.setMode(ExposureControl.Mode.Manual);
//        whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
//
//        // Apply initial values
//        exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
//        gainControl.setGain(GAIN);

        waitForStart();



        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(startPose)


                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(155))
                        .stopAndAdd(drivetrain.turnTo(-70,65))
//                        .stopAndAdd(vision.alignToTag(Vision.RED_GOAL_TAG_ID,-10))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity+5, bottomVelocity+5),
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
                        .strafeToSplineHeading(new Vector2d(35, 25), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(35, 60))
                        .stopAndAdd(transfer.tapTransfer())
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(155))
//                        .stopAndAdd(vision.alignToTag(Vision.RED_GOAL_TAG_ID,-10))
                        .stopAndAdd(drivetrain.turnTo(-70,65))
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

                        .splineToSplineHeading(new Pose2d(60, 20, Math.toRadians(90)), Math.toRadians(0))
                        .strafeTo(new Vector2d(60, 60))
                        .strafeTo(new Vector2d(60, 50))
                        .strafeTo(new Vector2d(60, 60))
                        .strafeTo(new Vector2d(60, 50))
                        .strafeTo(new Vector2d(60, 60))
                        .stopAndAdd(transfer.tapTransfer())
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(158))
//                        .stopAndAdd(vision.alignToTag(Vision.RED_GOAL_TAG_ID,4))
                        .stopAndAdd(drivetrain.turnTo(-70,65))
                        .stopAndAdd(
                                new ParallelAction(
                                        shooter.runShooter(topVelocity-10, bottomVelocity-10),
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
                        .strafeToLinearHeading(new Vector2d(50, 50), Math.toRadians(90))



                        .stopAndAdd(new SavePose())

                        .build());


    }


    public class SavePose implements InstantFunction {
        public void run() {
            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}
