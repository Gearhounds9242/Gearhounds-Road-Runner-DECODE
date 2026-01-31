package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

    @Override
    public void runOpMode() throws InterruptedException {
        // **************************************************
        // initialize hardware
        // **************************************************
        GearhoundsHardware robot = new GearhoundsHardware();
        robot.init(hardwareMap);  // Initialize robot (done ONCE)


        // **************************************************
        // initialize hardware mechanisms
        // **************************************************
        Pose2d startPose = new Pose2d(new Vector2d(60, 15), Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shooter shooter = new Shooter(robot);


        // **************************************************
        // define required trajectories
        // **************************************************
        TrajectoryActionBuilder gotoShootingPositionBuilder = drive.actionBuilder(startPose)
                .waitSeconds(10)
                .strafeToLinearHeading(new Vector2d(55, 15), Math.toRadians(163));

        // the following builder that includes 'endTrajectory.fresh' should always come at the end of a trajectory
        // so any new trajectories start fresh from the latest pose
        TrajectoryActionBuilder endGotoShootingPositionBuilder = gotoShootingPositionBuilder.endTrajectory().fresh();


        // **************************************************
        // build required Actions from trajectories
        // **************************************************
        Action gotoShootingPosition = gotoShootingPositionBuilder.build();
        Action endGotoShootingPosition = endGotoShootingPositionBuilder.build();


        // **************************************************
        // wait patiently for start
        // **************************************************
        waitForStart();
        if (isStopRequested()) return;


        // **************************************************
        // create and run action sequence
        // **************************************************
        Actions.runBlocking(new SequentialAction(
                        gotoShootingPosition,
                        new SleepAction(3),
                        shooter.runShooter(1400, 1400),
//                        shooter.shootBallRapid(1400, 1400, 3, 1.0, 4000), // timeout is in milliseconds
                        shooter.stopShooter(),
                        endGotoShootingPosition
                )
        ); // end Actions.runBlocking action sequence
    }


    /// Please don't go messing around in here if you don't know what you are doing proceed with CAUTION

//    public class SavePose implements InstantFunction {
//        @Override
//        public void run() {
//            PoseStorage.currentPose = drive.localizer.getPose();
//        }
//    }
}