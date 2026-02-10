package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Autonomous(name = "Run_Shooter")
public class Run_Shooter extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();
    MecanumDrive drive;
    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(-50, -50), Math.toRadians(146.25));


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot (done ONCE)
        robot.init(hardwareMap);

        // Create drive AFTER hardwareMap is ready
        drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(robot);
        Intake intake = new Intake(robot);

        waitForStart();

        if (isStopRequested()) return;
//        Action path = drive.actionBuilder(startPose)
//                .stopAndAdd(shooter.runShooter(800,800))
//                .build();
//        Actions.runBlocking(new SequentialAction(path));


        Actions.runBlocking(new SequentialAction(
                        shooter.runShooter(800, 800)
                )
        ); // end Actions.runBlocking action sequence

    }


    /// Please don't go messing around in here if you don't know what you are doing proceed with CAUTION


    public class SavePose implements InstantFunction {
        @Override
        public void run() {
            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}
