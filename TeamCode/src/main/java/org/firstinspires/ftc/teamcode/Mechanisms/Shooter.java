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




    // **************************************************
    //
    // **************************************************
    public class StopShooter implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.TopMotor.setPower(0);
            robot.BottomMotor.setPower(0);
            shooterDone = true;
            return false;
        }
    }
}
