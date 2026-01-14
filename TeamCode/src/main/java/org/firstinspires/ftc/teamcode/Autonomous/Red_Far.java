package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Autonomous(name = "Red_Far")
public class Red_Far extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();
    MecanumDrive drive;

    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(60, 15), Math.toRadians(180));


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot (done ONCE)
        robot.init(hardwareMap);

        // Create drive AFTER hardwareMap is ready
        drive = new MecanumDrive(hardwareMap, startPose);

        // Create drive AFTER hardwareMap is ready
        Shooter shooter = new Shooter(robot);

        waitForStart();

        if (isStopRequested()) return;
//        Actions.runBlocking(
//                drive.actionBuilder(startPose)
//                        .waitSeconds(10)
//                        .strafeToLinearHeading(new Vector2d(55, 15), Math.toRadians(163))
//                        .waitSeconds(3)
//                        .stopAndAdd(shooter.runShooter(1400, 1400, 0.2))
//                        .stopAndAdd(shooter.shootBallRapid(3, 1, 4))
//                        //                        .strafeToLinearHeading(new Vector2d(0,15),Math.toRadians(180))
//                        .build());

        Actions.runBlocking(new SequentialAction(
                        // drive.
//                        SleepAction(10),
                        // strafe
//                        SleepAction(10),
                        shooter.runShooter(1400, 1400, 0.2),
                        shooter.shootBallRapid(3, 1, 4)
                )
        );

    }


    /// Please don't go messing around in here if you don't know what you are doing proceed with CAUTION

    public class SavePose implements InstantFunction {
        @Override
        public void run() {
            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}