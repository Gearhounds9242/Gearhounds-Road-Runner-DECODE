
package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Autonomous(name = "Red_Far")
public class Red_Far extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();
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

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(55, 14, Math.toRadians(160.7)), Math.toRadians(0))
                        .waitSeconds(3)
//              add shoot
                        .splineTo(new Vector2d(35,30), Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(35, 60))
                        .stopAndAdd(intake.runIntake(1,5))
                .strafeTo(new Vector2d(35, 45))
                        .splineToSplineHeading(new Pose2d(55, 14, Math.toRadians(160.7)), Math.toRadians(0))
                        .waitSeconds(3)
//              add shoot
                        .splineTo(new Vector2d(12,30), Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(12, 60))
                .stopAndAdd(intake.runIntake(1,5))
        .strafeTo(new Vector2d(12, 45))
                        .splineToSplineHeading(new Pose2d(55, 14, Math.toRadians(160.7)), Math.toRadians(0))
                        .waitSeconds(3)
                        //              add shoot

                        .stopAndAdd(new SavePose())
                        .build());


    }


    /// Please don't go messing around in here if you don't know what you are doing proceed with CAUTION


    public class SavePose implements InstantFunction {
        @Override
        public void run() {PoseStorage.currentPose = drive.localizer.getPose();}
    }
}
