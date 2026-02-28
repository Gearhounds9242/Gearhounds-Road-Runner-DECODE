package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;


import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Disabled
@Autonomous(name = "Red_Far_9_Ball")
public class Red_Far_9_Ball extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();
    MecanumDrive drive;
    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(60, 15), Math.toRadians(180));

    PoseMap mirrorPoseMap = pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());

/*
You may see the word "Pose thrown around a lot. Pose is essentially just a way of explaining a point in space using coordinates like X and Y
 */

/// Some of the Autonomous commands include (these may change as time goes on will be updated here)
/*
.stopAndAdd(new SavePose())

.stopAndAdd(shooter.shootBallRapid("ballCount","transferWheelPower","timeout")
.stopAndAdd(shooter.runShooter("topPower","bottomPower", "timeout"))
.stopAndAdd(shooter.stopShooter())

.stopAndAdd(intake.runIntake("power","timeout"))
.stopAndAdd(intake.stopIntake())

.stopAndAdd(transfer.tapTransfer())
 */

//.stopAndAdd(
//         new ParallelAction(
//                 shooter.runShooter(800, 800),
//                 intake.runIntake(1, 0.1),
//                 new SequentialAction(
//                         new SleepAction(2),
//                         shooter.shootBallRapid(3, 1, 4)
//                 )
//         )
//)


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
        Actions.runBlocking(
                drive.actionBuilder(startPose)
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
