package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Autonomous(name = "Red_Far_Just_Move")
public class Red_Far_Just_Move extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();
    MecanumDrive drive;
    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(58, 14.5), Math.toRadians(180));


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot (done ONCE)
        robot.init(hardwareMap);

        // Create drive AFTER hardwareMap is ready
        drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .waitSeconds(15)
                        .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(360))
                        .stopAndAdd(new SavePose())
                        .build());


    }


    /// Please don't go messing around in here if you don't know what you are doing proceed with CAUTION


    public class SavePose implements InstantFunction {
        @Override
        public void run() {
            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}
