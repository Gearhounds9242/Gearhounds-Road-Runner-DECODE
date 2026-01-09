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
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.Autonomous_Comands.start_Shooter;

@Autonomous(name = "Red_Far")
public class Red_Far extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();

    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(60, 15), Math.toRadians(180));
    MecanumDrive drive;

    // ------------------------- //
    //      Instant Actions      //
    // ------------------------- //


    public class StartShooterWeak implements InstantFunction {
        @Override
        public void run() {
            robot.TopMotor.setVelocity(1680);
            robot.BottomMotor.setVelocity(1680);
        }
    }

    public class StartShooterStrong implements InstantFunction {
        @Override
        public void run() {
            robot.TopMotor.setVelocity(1720);
            robot.BottomMotor.setVelocity(1720);
        }
    }

    public class StopShooter implements InstantFunction {
        @Override
        public void run() {
            robot.TopMotor.setVelocity(0);
            robot.BottomMotor.setVelocity(0);
        }
    }



    public class ShootBall implements InstantFunction {
        @Override
        public void run() {
            // Drop -> Delay -> Reset
//            robot.drop.setPosition(0.28);
            Red_Far.this.sleep(200);   // 300 ms delay
//            robot.drop.setPosition(0.63);
        }
    }

    public class SavePose implements InstantFunction {
        @Override
        public void run() {
            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }

    public class DropUp implements InstantFunction{
        @Override
        public void run() {
//            robot.drop.setPosition(0.63);
            Red_Far.this.sleep(100);
        }
    }

    // ------------------------- //
    //        Main Auto          //
    // ------------------------- //
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot (done ONCE)
        robot.init(hardwareMap);

        // Create drive AFTER hardwareMap is ready
        drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();
        if (isStopRequested()) return;

        Action path = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(52,15))
                .splineToLinearHeading(new Pose2d(55, 18, Math.toRadians(160)), Math.toRadians(0))
                .stopAndAdd(new start_Shooter.StartShooter())
                .stopAndAdd(new SavePose())
                .build();

        Actions.runBlocking(new SequentialAction(path));
    }
}
