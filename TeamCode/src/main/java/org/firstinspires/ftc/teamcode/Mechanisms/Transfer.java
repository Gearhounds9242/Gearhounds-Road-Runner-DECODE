package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;

public class Transfer {

    private final GearhoundsHardware robot;

    public Transfer(GearhoundsHardware robot) {
        this.robot = robot;
    }

    public Action tapTransfer() {
        return new Transfer.tapTransfer();
    }

    public class tapTransfer implements Action {
        int currentPos;
        int targetPos;
        int howMuchToTap = 100;
        ElapsedTime timer;


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                currentPos = robot.transfer.getCurrentPosition();
                targetPos = currentPos + (howMuchToTap);
                robot.transfer.setTargetPosition(targetPos);
                robot.transfer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.transfer.setPower(1);
            }

            return true;
        }
    }
}
