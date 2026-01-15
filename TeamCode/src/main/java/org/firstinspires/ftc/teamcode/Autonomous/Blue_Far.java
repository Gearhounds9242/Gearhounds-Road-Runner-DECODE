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
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.PoseStorage;

@Autonomous(name = "Blue_Far")
public class Blue_Far extends LinearOpMode {

    MecanumDrive drive;
    private final GearhoundsHardware robot = new GearhoundsHardware();

    // Starting pose
    Pose2d startPose = new Pose2d(new Vector2d(60, -15), Math.toRadians(180));


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
        Action path = drive.actionBuilder(startPose)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(55,-15),Math.toRadians(-160))
                        .waitSeconds(1)
                .stopAndAdd(
                        new ParallelAction(
                                shooter.runShooter(800,800, 4),
                                new SequentialAction(
                                        new SleepAction(1),
                                        intake.runIntake(1,0.1),
                                        new SleepAction(2),
                                        shooter.shootBallRapid(800, 800, 3,1,4)
                                )
                        )
                )
//                        .stopAndAdd(
//                                new SequentialAction(
//                                        new ParallelAction(
//                                        shooter.runShooter(1400,1400,0.1),
//                                        intake.runIntake(1,0.1),
//                                        new SleepAction(10),
//                                        shooter.shootBallRapid(3,1,4)
//                                        )
//                                )
//
//                        )
                        .strafeToLinearHeading(new Vector2d(50,-20),Math.toRadians(180))

                        .build();
        Actions.runBlocking(new SequentialAction(path));


    }


    /// Please don't go messing around in here if you don't know what you are doing proceed with CAUTION


    public class SavePose implements InstantFunction {
        @Override
        public void run() {
            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }


    public class ShootBallRapid implements Action {
        int ballNumber;
        double transferPower;
        int currentPos;
        int howMuchToSpinPerBall = 10000;
        int targetPos;
        int timeout;
        ElapsedTime timer;

        public ShootBallRapid(int ballCount, int power, int timeout) {
            this.ballNumber = ballCount;
            this.transferPower = power;
            this.timeout = timeout;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                currentPos = robot.transfer.getCurrentPosition();
                targetPos = currentPos + (ballNumber * howMuchToSpinPerBall);
                robot.transfer.setTargetPosition(targetPos);
                robot.transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (transferPower > 1 || transferPower < -1 || transferPower == 0) {
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                throw new RuntimeException("The transfer uses .setPower! The power must be less then 1, but not 0 or below (it can be a decimal). Change to continue");
            }
            if (timeout <= 0) {
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                throw new RuntimeException("The transfer timeout needs to be greater than 0!");
            }

            robot.transfer.setTargetPosition(targetPos);
            robot.transfer.setPower(transferPower);

            if (robot.transfer.getCurrentPosition() >= targetPos) {
                robot.transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
            if (timer.seconds() < timeout) {
                return true;
            } else {
                robot.transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                return false;
            }
        }
    }

    public class StopShooter implements Action {
        double Top_Target_Speed = 0;
        double Bottom_Target_Speed = 0;
        ElapsedTime timer;

        PIDFController topShooterController = new PIDFController(top_P, top_I, top_D, top_F);
        PIDFController bottomShooterController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);

        double topOutput = topShooterController.calculate(robot.TopMotor.getVelocity(), Top_Target_Speed);
        double bottomOutput = bottomShooterController.calculate(robot.BottomMotor.getVelocity(), Bottom_Target_Speed);

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.TopMotor.setVelocity(topOutput);
            robot.BottomMotor.setVelocity(bottomOutput);
            return false;
        }
    }

    public class runIntake implements Action {
        double power;
        double timeout;
        ElapsedTime timer;


        public runIntake(double Power, double Timeout) {
            this.power = Power;
            this.timeout = Timeout;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                //put code to initialize here per action
            }
            if (power == 0) {
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                throw new RuntimeException("The intake power can not be 0");
            }
            if (power < -1 || power > 1) {
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                throw new RuntimeException("The intake uses .setPower! Can not be above or below 1 or -1 Change to continue");
            }
            if (timeout <= 0) {
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                throw new RuntimeException("The intake timeout needs to be greater than 0!");
            }
            robot.intake.setPower(power);


            if (timer.seconds() < timeout) {
                return true;
            } else {
                return false;
            }
        }
    }
    public class RunShooter implements Action {
        double Top_Target_Speed;
        double Bottom_Target_Speed;
        double timeout;
        ElapsedTime timer;

        PIDFController topShooterController = new PIDFController(top_P, top_I, top_D, top_F);
        PIDFController bottomShooterController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);

        public RunShooter(double topPower, double bottomPower, double timeout) {
            this.Top_Target_Speed = topPower;
            this.Bottom_Target_Speed = bottomPower;
            this.timeout = timeout;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                //put code to initialize here per action
            }
            if (Top_Target_Speed < 0 || Bottom_Target_Speed < 0) {
                throw new RuntimeException("The shooter power needs to be a positive number! Change to continue");
            }
            if (timeout <= 0) {
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                throw new RuntimeException("The shooter timeout needs to be greater than 0!");
            }
            double topOutput = topShooterController.calculate(robot.TopMotor.getVelocity(), Top_Target_Speed);
            double bottomOutput = bottomShooterController.calculate(robot.BottomMotor.getVelocity(), Bottom_Target_Speed);
            robot.TopMotor.setVelocity(topOutput);
            robot.BottomMotor.setVelocity(bottomOutput);

            if(robot.TopMotor.getVelocity() >= Top_Target_Speed && robot.BottomMotor.getVelocity() >= Top_Target_Speed){
                return true;
            }

            if (timer.seconds() < timeout) {
                return true;
            } else {
                return false;
            }
        }
    }

}
