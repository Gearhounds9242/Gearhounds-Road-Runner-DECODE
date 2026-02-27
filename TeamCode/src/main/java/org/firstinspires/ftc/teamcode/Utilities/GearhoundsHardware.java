package org.firstinspires.ftc.teamcode.Utilities;

//import com.qualcomm.hardware.bosch.BNO055IMU i

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;



// Generic robot class
public class GearhoundsHardware extends Hardware {
    public HardwareMap robotMap;

    // Examples Here
/*  public DcMotorEx ExampleMotor;
    public Servo ExampleServo;
    public HuskyLens huskyLens;
*/
//    public Servo drop;
    public DcMotorEx intake;
    public DcMotorEx topMotor;
    public DcMotorEx bottomMotor;
    public RevBlinkinLedDriver blinkin;

    //Comment back in when needed for vision
    public WebcamName webcam;
    //Drivetrain Members
    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;

    public DcMotorEx transfer;
    public Servo block;
    public Servo leftLight;
    public Servo rightLight;




//    public DcMotorEx motor;


    public static final int READ_PERIOD = 1;
    public IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    private YawPitchRollAngles             lastAngles;
    private static double                  globalAngle;

    // 1000 ticks was roughly 18 in.
    public static final double TICK_PER_INCH = 600.0/12.0;
    public static final double MinPower = 0.3;

    static final double FEET_PER_METER = 3.28084;

    //Constructor
    public GearhoundsHardware() {

    }

    // Override to set actual robot configuration
    public void init(HardwareMap hwMap) {

        // Save reference to Hardware map
        robotMap = hwMap;
        //Examples Here
    /*
        huskyLens = robotMap.get(HuskyLens.class, "huskyLens");
        ExampleServo = robotMap.get(Servo.class, "ExampleServo");
        ExampleMotor = robotMap.get(DcMotorEx.class, "ExampleMotor");
        NameInCodeHere = robotMap.get(DeviceTypeHere.class, "DriverStationNameHere");
    */
//        blinkin = robotMap.get(RevBlinkinLedDriver.class, "blinkin");

//Comment back in when needed for vision
        webcam = robotMap.get(WebcamName.class, "Webcam 1");
        //Drivetrain Members

//        drop = robotMap.get(Servo.class, "drop");

//        motor = robotMap.get(DcMotorEx.class, "motor");
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor
        block = robotMap.get(Servo.class,"block");
        leftLight = robotMap.get(Servo.class,"leftLight");
        rightLight = robotMap.get(Servo.class,"rightLight");

        topMotor = robotMap.get(DcMotorEx.class, "topmotor");
        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomMotor = robotMap.get(DcMotorEx.class, "bottommotor");
        bottomMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        leftFront = robotMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack = robotMap.get(DcMotorImplEx.class, "leftBack");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront = robotMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack = robotMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);



        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        transfer = robotMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);



        intake = robotMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorImplEx.Direction.REVERSE);

        // Defines the REV Hub's internal IMU (Gyro)
        imu = robotMap.get(IMU.class, "imu");

        // Defines the parameters for the gyro (units)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParameters);
        resetAngle();
    }



    public void resetAngle() {
        imu.resetYaw();
        lastAngles = imu.getRobotYawPitchRollAngles();
        globalAngle = 0;
    }

    public double getAngle() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double deltaAngle = angles.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
    public double clamp( double x, double min, double max) {return Math.max(min,Math.min(max,x));}
}
