package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.Bottom_Target_Speed;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.Top_Target_Speed;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_D;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_F;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_I;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_P;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_D;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_F;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_I;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_P;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;
@Disabled
@Autonomous(name = "Red_Far")
public class Red_Far_Old extends LinearOpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();

    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(60, 15), Math.toRadians(180));
    MecanumDrive drive;

    // ------------------------- //
    //      Instant Actions      //
    // ------------------------- //



    public class StartShooter implements InstantFunction {
        @Override
        public void run() {
            robot.TopMotor.setVelocity(Top_Target_Speed);
            robot.BottomMotor.setVelocity(Bottom_Target_Speed);
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
            Red_Far_Old.this.sleep(200);   // 300 ms delay
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
            Red_Far_Old.this.sleep(100);
        }
    }

    // ------------------------- //
    //        Main Auto          //
    // ------------------------- //



    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot (done ONCE)
        robot.init(hardwareMap);

        PIDFController topShooterController = new PIDFController(top_P, top_I, top_D, top_F);
        PIDFController bottomShooterController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);

        // Create drive AFTER hardwareMap is ready
        drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();
        if (isStopRequested()) return;
        double topOutput = topShooterController.calculate(robot.TopMotor.getVelocity(),Top_Target_Speed);
        double bottomOutput = bottomShooterController.calculate(robot.BottomMotor.getVelocity(),Bottom_Target_Speed);

        Action path = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(52,15))
                .splineToLinearHeading(new Pose2d(55, 18, Math.toRadians(160)), Math.toRadians(0))
                .stopAndAdd(new StartShooter())
                .stopAndAdd(new SavePose())
                .build();

        Actions.runBlocking(new SequentialAction(path));
    }


}
