//package org.firstinspires.ftc.teamcode.Mechanisms;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//
//public class Intake {
//
//    private final DcMotorEx intake;
//
//    public Intake(final HardwareMap hMap, final String name) {
//        intake = hMap.get(DcMotorEx.class, "intake");
//        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//
//    public double getIntakePower(){
//        double intakePower = intake.getPower();
//        return intakePower;
//    }
//
//    public void intakeOff(){
//        intake.setPower(0);
//    }
//
//    public void intakeOn(){
//        intake.setPower(1);
//    }
//
//    public void intakeReverse(){
//        intake.setPower(-1);
//    }
//}
