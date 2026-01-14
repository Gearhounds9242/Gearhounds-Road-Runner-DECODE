//package org.firstinspires.ftc.teamcode.Autonomous_Comands;
//
//import com.acmerobotics.roadrunner.InstantFunction;
//import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
//
//public class start_Shooter implements InstantFunction {
//
//    private final GearhoundsHardware robot;
//    private final double velocity;
//
//    public StartShooter(GearhoundsHardware robot, double velocity) {
//        this.robot = robot;
//        this.velocity = velocity;
//    }
//
//    @Override
//    public void run() {
//        robot.TopMotor.setVelocity(velocity);
//        robot.BottomMotor.setVelocity(velocity);
//    }
//}
