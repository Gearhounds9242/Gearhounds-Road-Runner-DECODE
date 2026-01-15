package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;

import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_D;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_F;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_I;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.bottom_P;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_D;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_F;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_I;
import static org.firstinspires.ftc.teamcode.TeleOp.Mechanum.top_P;


public class Shooter {

    private final GearhoundsHardware robot;

    public Shooter(GearhoundsHardware robot) {
        this.robot = robot;
    }

    public Action runShooter(double topPower, double bottomPower) {
        return new RunShooter(topPower, bottomPower);
    }

    public Action stopShooter() {
        return new StopShooter();
    }

    public Action shootBallRapid(int ballCount, double power, int timeout) {
        return new ShootBallRapid(ballCount, power, timeout);
    }


    public class RunShooter implements Action {
        private boolean initialized = false;
        double topRollerTargetVelocity;
        double bottomRollerTargetVelocity;
        PIDFController topShooterController = new PIDFController(top_P, top_I, top_D, top_F);
        PIDFController bottomShooterController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);

        public RunShooter(double topRollerTargetVelocity, double bottomRollerTargetVelocity) {
            this.topRollerTargetVelocity = topRollerTargetVelocity;
            this.bottomRollerTargetVelocity = bottomRollerTargetVelocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (topRollerTargetVelocity < 0 || bottomRollerTargetVelocity < 0) {
                throw new RuntimeException("The shooter power needs to be a positive number! Change to continue");
            }

            double topOutput = topShooterController.calculate(robot.TopMotor.getVelocity(), topRollerTargetVelocity);
            double bottomOutput = bottomShooterController.calculate(robot.BottomMotor.getVelocity(), bottomRollerTargetVelocity);
            robot.TopMotor.setVelocity(topOutput);
            robot.BottomMotor.setVelocity(bottomOutput);

            double topMotorVelocityCurrentVelocity = robot.TopMotor.getVelocity();
            double bottomMotorVelocityCurrentVelocity = robot.BottomMotor.getVelocity();

            if (topMotorVelocityCurrentVelocity >= topRollerTargetVelocity && bottomMotorVelocityCurrentVelocity >= bottomRollerTargetVelocity) {
                return false;
            } else {
                return true;
            }
        }
    }


    public class StopShooter implements Action {
        double topRollerTargetVelocity = 0;
        double bottomRollerTargetVelocity = 0;

        PIDFController topShooterController = new PIDFController(top_P, top_I, top_D, top_F);
        PIDFController bottomShooterController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);

        double topRollerVelocity = topShooterController.calculate(robot.TopMotor.getVelocity(), topRollerTargetVelocity);
        double bottomRollerVelocity = bottomShooterController.calculate(robot.BottomMotor.getVelocity(), bottomRollerTargetVelocity);

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.TopMotor.setVelocity(topRollerVelocity);
            robot.BottomMotor.setVelocity(bottomRollerVelocity);
            return false;
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

        public ShootBallRapid(int ballCount, double power, int timeout) {
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
}
