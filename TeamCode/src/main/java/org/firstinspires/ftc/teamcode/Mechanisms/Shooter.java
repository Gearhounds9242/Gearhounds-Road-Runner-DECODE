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
    // CONSTANTS
    private static final double ROLLER_VELOCITY_TOLERANCE = 50;  // this is ticks/sec
    private static final int STABLE_ROLLER_LOOPS_REQUIREMENT = 5; // how many passes through 'run' must rollers READY before completing Action
    private static final int SPIN_PER_BALL = 10000; // no idea what these units are ... some made-up Harry unit
    private final GearhoundsHardware robot;

    public Shooter(GearhoundsHardware robot) {
        this.robot = robot;
    }

    // **************************************************
    //
    // **************************************************
    public Action runShooter(double topRollerTargetVelocity, double bottomRollerTargetVelocity) {
        return new RunShooter(topRollerTargetVelocity, bottomRollerTargetVelocity);
    }

    public Action shootBallRapid(double topRollerTargetVelocity, double bottomRollerTargetVelocity, int ballCount, double transferBeltPower, int timeoutMs) {
        return new ShootBallRapid(topRollerTargetVelocity, bottomRollerTargetVelocity, ballCount, transferBeltPower, timeoutMs);
    }

    public Action stopShooter() {
        return new StopShooter();
    }


    // **************************************************
    //
    // **************************************************
    public class RunShooter implements Action {
        private final PIDFController topRollerController;
        private final PIDFController bottomRollerController;
        private final double topRollerTargetVelocity;
        private final double bottomRollerTargetVelocity;
        private int stableRollerLoops = 0;

        public RunShooter(double topRollerTargetVelocity, double bottomRollerTargetVelocity) {
            // confirm velocity values top and bottom rollers
            if (topRollerTargetVelocity < 0 || bottomRollerTargetVelocity < 0) {
                throw new IllegalArgumentException("The shooter power needs to be a positive number! Change to continue");
            }
            this.topRollerController = new PIDFController(top_P, top_I, top_D, top_F);
            this.bottomRollerController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);
            this.topRollerTargetVelocity = topRollerTargetVelocity;
            this.bottomRollerTargetVelocity = bottomRollerTargetVelocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // get roller velocities
            double topRollerVelocity = robot.TopMotor.getVelocity();
            double bottomRollerVelocity = robot.BottomMotor.getVelocity();

            // update motor speeds with PIDF
            double topOutput = topRollerController.calculate(topRollerVelocity, topRollerTargetVelocity);
            double bottomOutput = bottomRollerController.calculate(bottomRollerVelocity, bottomRollerTargetVelocity);
            robot.TopMotor.setVelocity(topOutput);
            robot.BottomMotor.setVelocity(bottomOutput);

            // check to see how close we are to target velocity
            boolean topRollerReady = Math.abs(topRollerTargetVelocity - topRollerVelocity) < ROLLER_VELOCITY_TOLERANCE;
            boolean bottomRollerReady = Math.abs(bottomRollerTargetVelocity - bottomRollerVelocity) < ROLLER_VELOCITY_TOLERANCE;

            // log some info to see what this code actually does 🤷🏻‍♂️
//            telemetryPacket.put("Top Target Velocity", topRollerVelocity);
//            telemetryPacket.put("  Top Velocity", topRollerVelocity);
//            telemetryPacket.put("  Top Ready", topRollerReady);
//            telemetryPacket.put("Bottom Target Velocity", topRollerVelocity);
//            telemetryPacket.put("  Bottom Velocity", bottomRollerVelocity);
//            telemetryPacket.put("  Bottom Ready", bottomRollerReady);
//            telemetryPacket.put("Stable Loops", stableRollerLoops);

            // only exit if rollers have been stable for STABLE_ROLLER_LOOPS_REQUIREMENT
            if (topRollerReady && bottomRollerReady) {
                stableRollerLoops++;
            } else {
                stableRollerLoops = 0;
            }
            return stableRollerLoops < STABLE_ROLLER_LOOPS_REQUIREMENT;
        }
    }



    // **************************************************
    //
    // **************************************************
    public class ShootBallRapid implements Action {
        private final PIDFController topRollerController;
        private final PIDFController bottomRollerController;
        private final double topRollerTargetVelocity;
        private final double bottomRollerTargetVelocity;
        private final int ballCount;
        private final double transferWheelPower;
        private final int timeoutMs;
        int currentPos;
        int targetPos;
        ElapsedTime timer;

        public ShootBallRapid(double topRollerTargetVelocity, double bottomRollerTargetVelocity, int ballCount, double transferWheelPower, int timeoutMs) {
            // confirm velocity values top and bottom rollers
            if (topRollerTargetVelocity < 0 || bottomRollerTargetVelocity < 0) {
                throw new IllegalArgumentException("The shooter power needs to be a positive number! Change to continue");
            }
            this.topRollerTargetVelocity = topRollerTargetVelocity;
            this.bottomRollerTargetVelocity = bottomRollerTargetVelocity;

            // confirm value input for transferWheelPower
            if (transferWheelPower > 1 || transferWheelPower < -1 || transferWheelPower == 0) {
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                throw new RuntimeException("The transfer uses .setPower! The power must be less then 1, but not 0 or below (it can be a decimal). Change to continue");
            }
            // confirm value input for timeout
            if (timeoutMs <= 0) {
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                throw new RuntimeException("The transfer timeout needs to be greater than 0!");
            }
            this.topRollerController = new PIDFController(top_P, top_I, top_D, top_F);
            this.bottomRollerController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);
            this.ballCount = ballCount;
            this.transferWheelPower = transferWheelPower;
            this.timeoutMs = timeoutMs;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                currentPos = robot.transfer.getCurrentPosition();
                targetPos = currentPos + (ballCount * SPIN_PER_BALL);
                robot.transfer.setTargetPosition(targetPos);
                robot.transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.transfer.setPower(transferWheelPower);
            }

            // get roller velocities
            double topRollerVelocity = robot.TopMotor.getVelocity();
            double bottomRollerVelocity = robot.BottomMotor.getVelocity();

            // update motor speeds with PIDF
            double topOutput = topRollerController.calculate(topRollerVelocity, topRollerTargetVelocity);
            double bottomOutput = bottomRollerController.calculate(bottomRollerVelocity, bottomRollerTargetVelocity);
            robot.TopMotor.setVelocity(topOutput);
            robot.BottomMotor.setVelocity(bottomOutput);

            // log some info to see what this code actually does 🤷🏻‍♂️
//            telemetryPacket.put("Top Velocity", topRollerVelocity);
//            telemetryPacket.put("Bottom Velocity", bottomRollerVelocity);
//            telemetryPacket.put("Transfer Busy", robot.transfer.isBusy());

            // exit when robot.transfer reaches correct position
            // or when the timer exceeds the timeout
            if (!robot.transfer.isBusy() || timer.milliseconds() > timeoutMs) {
                robot.transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.transfer.setPower(0); // shut down transfer on exit
                robot.leftLight.setPosition(0.28);
                robot.rightLight.setPosition(0.28);
                return false;
            }
            return true;
        }
    }



    // **************************************************
    //
    // **************************************************
    public class StopShooter implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.TopMotor.setVelocity(0);
            robot.BottomMotor.setVelocity(0);
            return false;
        }
    }
}
