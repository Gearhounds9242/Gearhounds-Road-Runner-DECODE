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
    private static final double ROLLER_VELOCITY_TOLERANCE = 5000000;  // this is ticks/sec
    private static final int STABLE_ROLLER_LOOPS_REQUIREMENT = 10; // how many passes through 'run' must rollers READY before completing Action
    private static final int SPIN_PER_BALL = 10000; // no idea what these units are ... some made-up Harry unit
    private final GearhoundsHardware robot;
    private final PIDFController topRollerController;
    private final PIDFController bottomRollerController;
    private boolean shooterDone;

    public Shooter(GearhoundsHardware robot) {
        this.robot = robot;
        this.topRollerController = new PIDFController(top_P, top_I, top_D, top_F);
        this.bottomRollerController = new PIDFController(bottom_P, bottom_I, bottom_D, bottom_F);
    }

    public Action runShooter(double topRollerTargetVelocity, double bottomRollerTargetVelocity) {
        return new RunShooter(topRollerTargetVelocity, bottomRollerTargetVelocity);
    }

    public Action shootSequence(double topVelocity, double bottomVelocity,
                                int ballCount, double transferPower, int timeoutSec) {
        return new Shooter.ShootSequence(topVelocity, bottomVelocity, ballCount, transferPower, timeoutSec);
    }
    public Action stopShooter() {
        return new StopShooter();
    }


    // **************************************************
    //
    // **************************************************
    public class RunShooter implements Action {
        private final double topRollerTargetVelocity;
        private final double bottomRollerTargetVelocity;

        public RunShooter(double topRollerTargetVelocity, double bottomRollerTargetVelocity) {
            // confirm velocity values top and bottom rollers
            if (topRollerTargetVelocity < 0 || bottomRollerTargetVelocity < 0) {
                throw new RuntimeException("The shooter power needs to be a positive number! Change to continue");
            }

            this.topRollerTargetVelocity = topRollerTargetVelocity;
            this.bottomRollerTargetVelocity = bottomRollerTargetVelocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // get roller velocities
            double topVelocity = robot.TopMotor.getVelocity();
            double bottomVelocity = robot.BottomMotor.getVelocity();

            // update motor speeds with PIDF
            double topOutput = topRollerController.calculate(topVelocity, topRollerTargetVelocity);
            double bottomOutput = bottomRollerController.calculate(bottomVelocity, bottomRollerTargetVelocity);
            robot.TopMotor.setVelocity(topOutput);
            robot.BottomMotor.setVelocity(bottomOutput);




            if (shooterDone == true){
                shooterDone = false;
                return false;
            }else {
                return true;
            }
        }
    }

    public class ShootSequence implements Action {
        private final double topTarget;
        private final double bottomTarget;
        private final int ballCount;
        private final double transferPower;
        private final float timeoutSec;

        private ElapsedTime timer;
        private int stableLoops = 0;
        private boolean shooterReady = false;
        private boolean transferStarted = false;
        private int transferStartPos;
        private int transferTargetPos;

        public ShootSequence(double topTarget, double bottomTarget,
                             int ballCount, double transferPower, int timeoutSec) {
            // Input validation
            if (topTarget < 0 || bottomTarget < 0) {
                throw new RuntimeException("Shooter velocities must be positive!");
            }
            if (transferPower > 1 || transferPower < -1 || transferPower == 0) {
                throw new RuntimeException("Transfer power must be between -1 and 1, and not 0!");
            }
//            if (transferTimeoutSec <= 0) {
//                throw new RuntimeException("Transfer timeout must be greater than 0!");
//            }

            this.topTarget = topTarget;
            this.bottomTarget = bottomTarget;
            this.ballCount = ballCount;
            this.transferPower = transferPower;
            this.timeoutSec = timeoutSec;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

//            // Always maintain shooter velocity using shared PIDF controllers
            double topVel = robot.TopMotor.getVelocity();
            double bottomVel = robot.BottomMotor.getVelocity();
//            double topOutput = topRollerController.calculate(topVel, topTarget);
//            double bottomOutput = bottomRollerController.calculate(bottomVel, bottomTarget);

            robot.TopMotor.setVelocity(topTarget);
            robot.BottomMotor.setVelocity(bottomTarget);

            // Phase 1: Spin up and wait for stable velocity
//            if (!shooterReady) {
//                boolean topReady = Math.abs(topTarget - topVel) < ROLLER_VELOCITY_TOLERANCE;
//                boolean bottomReady = Math.abs(bottomTarget - bottomVel) < ROLLER_VELOCITY_TOLERANCE;
//
//                if (topReady && bottomReady) {
//                    stableLoops++;
//                } else {
//                    stableLoops = 0;
//                }
//
//                if (stableLoops >= STABLE_ROLLER_LOOPS_REQUIREMENT) {
//                    shooterReady = true;
//                }
//                return true; // Keep spinning up
//            }
//
//            // Phase 2: Start transfer to shoot balls
//            if (!transferStarted) {
//                transferStartPos = robot.transfer.getCurrentPosition();
//                transferTargetPos = transferStartPos + (ballCount * SPIN_PER_BALL);
//                robot.transfer.setTargetPosition(transferTargetPos);
//                robot.transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.transfer.setPower(transferPower);
//                transferStarted = true;
//            }
//may not need this
//             Phase 3: Keep transfer running until complete
//            robot.transfer.setPower(transferPower);

//            // Check if shooting is complete
//            if (!robot.transfer.isBusy() || timer.seconds() > transferTimeoutSec) {
//                // Phase 4: Clean up - stop all motors
//                robot.transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.transfer.setPower(0);
//                robot.TopMotor.setPower(0);
//                robot.BottomMotor.setPower(0);
//                return true; // Action complete
//            }
            return !(timer.seconds() > timeoutSec);// Still shooting
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
            shooterDone = true;
            return false;
        }
    }
}
