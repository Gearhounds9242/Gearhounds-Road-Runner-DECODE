package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;


public class Intake {

    private final GearhoundsHardware robot;

    public Intake(GearhoundsHardware robot) {
        this.robot = robot;
    }

    public Action runIntake(double power, double timeout) {
        return new RunIntake(power, timeout);
    }

    public Action stopShooter() {
        return new StopIntake();
    }



    public class RunIntake implements Action {
        double power;
        double timeout;
        ElapsedTime timer;


        public RunIntake(double Power, double Timeout) {
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


    public class StopIntake implements Action {
        double power = 0;
        ElapsedTime timer;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.intake.setPower(power);
            return false;
        }
    }
}
