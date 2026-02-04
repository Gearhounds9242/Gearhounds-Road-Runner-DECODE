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
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Disabled
/// ATTENTION IF YOU ARE GOING TO COPY THIS FILE @Disabled MUST BE REMOVED FOR IT TO SHOW UP ON THE DRIVER STATION
@Autonomous(name = "Autonomous_Example")
///  ATTENTION IF YOU ARE GOING TO COPY THIS FILE BOTH OF THE NAMES SHOWN HERE AND ON THE LINE BELOW MUST MATCH YOUR FILE NAME
public class Autonomous_Example extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();
    MecanumDrive drive;
    /// ********************************************
    /// This is your starting position. It is important to get this correct as it is one of the main ways autonomous gets messed up
    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(60, 15), Math.toRadians(180));

    /// When you type this next to the startPose in "drive.actionBuilder(startPose "RIGHT HERE")" it will flip all the paths so it will work on the other side the only thing you have to change is the start pose otherwise it thinks its in timbucktoo
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

/// On of the main commands you may want to use is to shoot. Here is the current code to shoot 3 balls.
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

    /// ********************************************

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
/// Right here is where you will start your pathing, before the .stopAndAdd(new SavePose()). All GearHounds Autonomous pathing end with .stopAndAdd(new SavePose()). The .build()); is just required for Roadrunner.
                        .stopAndAdd(new SavePose())
                        .build());


    }


    /// Here are is the "SavePose" command, this command allows for the robot to remember where it is after auto ends.
    /// For the rest of the commands look inside the folder org.firstin...teamcode/Mechanisms
// TODO: 1/30/2026 eventually there needs to be an example file for creating mechanisms
// TODO: 2/4/20206 need to make example of how to flip the auto for the other side


    public class SavePose implements InstantFunction {
        @Override
        public void run() {
            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}
