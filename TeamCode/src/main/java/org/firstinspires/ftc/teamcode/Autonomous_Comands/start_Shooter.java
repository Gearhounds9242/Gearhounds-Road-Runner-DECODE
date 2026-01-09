package org.firstinspires.ftc.teamcode.Autonomous_Comands;



import com.acmerobotics.roadrunner.InstantFunction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;

public class start_Shooter {

    private final GearhoundsHardware robot = new GearhoundsHardware();

    public class StartShooter implements InstantFunction {
        @Override
        public void run() {
            robot.TopMotor.setVelocity(1690);
            robot.BottomMotor.setVelocity(1690);
        }
    }





}
