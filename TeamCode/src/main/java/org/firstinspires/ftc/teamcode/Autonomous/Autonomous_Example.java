package org.firstinspires.ftc.teamcode.Autonomous;


import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_D;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_F;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_I;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_P;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_D;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_F;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_I;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_P;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Disabled/// ATTENTION IF YOU ARE GOING TO COPY THIS FILE @Disabled MUST BE REMOVED FOR IT TO SHOW UP ON THE DRIVER STATION
@Autonomous(name = "Autonomous_Example")///  ATTENTION IF YOU ARE GOING TO COPY THIS FILE BOTH OF THE NAMES SHOWN HERE AND ON THE LINE BELOW MUST MATCH YOUR FILE NAME
public class Autonomous_Example extends LinearOpMode {

MecanumDrive drive;
private final GearhoundsHardware robot = new GearhoundsHardware();

/// ********************************************
///This is your starting position. It is important to get this correct as it is one of the main ways autonomous gets messed up
    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(60, 15), Math.toRadians(180));

/*
You may see the word "Pose thrown around a lot. Pose is essentially just a way of explaining a point in space using coordinates like X and Y
 */

/// Some of the Autonomous commands include (these may change as time goes on will be updated here)
/*
.stopAndAdd(new SavePose())
.stopAndAdd(new ShootBallRapid("ballCount","power","timeout")
.stopAndAdd(new RunShooter("topPower","bottomPower", "timeout"))
.stopAndAdd(new StopShooter())
 */
/// ********************************************

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
/// Right here is where you will start your pathing, before the .stopAndAdd(new SavePose()). All GearHounds Autonomous pathing end with .stopAndAdd(new SavePose()). The .build()); is just required for Roadrunner.
                        .stopAndAdd(new SavePose())
                        .build());


    }


    /// Here are all of your Autonomous commands. you can use these by typing .stopAndAdd(new "command name here")
    /// Please don't go messing around in here if you don't know what you are doing proceed with CAUTION


    public class SavePose implements InstantFunction {
        @Override
        public void run() {
            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }


    public class ShootBallRapid implements Action{
        int ballNumber;
        double transferPower;
        int currentPos;
        int howMuchToSpinPerBall = 100;
        int targetPos;
        int timeout;
        ElapsedTime timer;

        public ShootBallRapid(int ballCount, int power, int timeout){
            this.ballNumber = ballCount;
            this.transferPower = power;
            this.timeout = timeout;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null){
                timer = new ElapsedTime();
                currentPos = robot.transfer.getCurrentPosition();
                robot.transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                targetPos = currentPos + (ballNumber * howMuchToSpinPerBall);
            }
            if (transferPower > 1 || transferPower < -1 || transferPower == 0){
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                throw new RuntimeException("The transfer uses .setPower! The power must be less then 1, but not 0 or below (it can be a decimal). Change to continue");
            }
            if(timeout <= 0){
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                throw new RuntimeException("The transfer timeout needs to be greater than 0!");
            }

            robot.transfer.setTargetPosition(targetPos);
            robot.transfer.setPower(transferPower);

            if(robot.transfer.getCurrentPosition() >= targetPos){
                robot.transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
            if (timer.seconds() < timeout){
                return true;
            } else {
                robot.transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                return false;
            }
        }
    }

    public class StopShooter implements Action{
        double Top_Target_Speed = 0;
        double Bottom_Target_Speed = 0;
        ElapsedTime timer;

        PIDFController topShooterController = new PIDFController(top_P, top_I, top_D, top_F);
        PIDFController bottomShooterController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);

        double topOutput = topShooterController.calculate(robot.TopMotor.getVelocity(),Top_Target_Speed);
        double bottomOutput = bottomShooterController.calculate(robot.BottomMotor.getVelocity(),Bottom_Target_Speed);

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.TopMotor.setVelocity(topOutput);
            robot.BottomMotor.setVelocity(bottomOutput);
            return false;
        }
    }

    public class RunShooter implements Action{
        double Top_Target_Speed;
        double Bottom_Target_Speed;
        //        int spoolTime;
        int timeout;
        ElapsedTime timer;

        PIDFController topShooterController = new PIDFController(top_P, top_I, top_D, top_F);
        PIDFController bottomShooterController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);

        double topOutput = topShooterController.calculate(robot.TopMotor.getVelocity(),Top_Target_Speed);
        double bottomOutput = bottomShooterController.calculate(robot.BottomMotor.getVelocity(),Bottom_Target_Speed);

        public RunShooter(double topPower, double bottomPower/*, int spoolTime*/, int timeout){
            this.Top_Target_Speed = topPower;
            this.Bottom_Target_Speed = bottomPower;
//            this.spoolTime = spoolTime;
            this.timeout = timeout;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null){
                timer = new ElapsedTime();
                //put code to initialize here per action
            }
            if (Top_Target_Speed < 0 || Bottom_Target_Speed < 0){
                throw new RuntimeException("The shooter power needs to be a positive number! Change to continue");
            }
            if(timeout <= 0){
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                throw new RuntimeException("The shooter timeout needs to be greater than 0!");
            }
            robot.TopMotor.setVelocity(topOutput);
            robot.BottomMotor.setVelocity(bottomOutput);

            if(robot.TopMotor.getVelocity() >= Top_Target_Speed && robot.BottomMotor.getVelocity() >= Top_Target_Speed){
                return true;
            }
            if (timer.seconds() < timeout){
                return true;
            } else {
                return false;
            }
        }
    }

}
