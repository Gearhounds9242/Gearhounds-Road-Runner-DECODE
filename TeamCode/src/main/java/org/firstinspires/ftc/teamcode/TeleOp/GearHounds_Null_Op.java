package org.firstinspires.ftc.teamcode.TeleOp;//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.controller.PController;
//import com.seattlesolvers.solverslib.controller.PIDController;
//import com.seattlesolvers.solverslib.controller.PIDFController;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
//
//
//@Config
//@TeleOp(name = "GearHounds NullOp", group = "Concept")
//public class GearHounds_Null_Op extends OpMode {
//    private final GearhoundsHardware robot = new GearhoundsHardware();
//    private ElapsedTime runtime = new ElapsedTime();
//    private FtcDashboard dashboard;
////  public static double "variable here"
//    public static double kP = 0;
//    public static double kI = 0;
//    public static double kD = 0;
//    public static double kF = 0;
//
//    public static double motorTargetPosition = 0;
//    public static double reference = 0;
//    public static double integralSum = 0;
//    public static double lastError = 0;
//    public static double encoderPosition = 0;
//
//
//    PIDController motor = new PIDController(kP, kI, kD);
//
//    /**
//     * This method will be called once, when the INIT button is pressed.
//     */
//    @Override
//    public void init() {
//        dashboard = FtcDashboard.getInstance();
//        dashboard.setTelemetryTransmissionInterval(25);
//        robot.init(hardwareMap);
//        telemetry.addData("Status", "Initialized");
//
//        motor.setPID(kP, kI, kD);
//    }
//
//    /**
//     * This method will be called repeatedly during the period between when
//     * the INIT button is pressed and when the START button is pressed (or the
//     * OpMode is stopped).
//     */
//    @Override
//    public void init_loop() {
//    }
//
//    /**
//     * This method will be called once, when the START button is pressed.
//     */
//    @Override
//    public void start() {
//        runtime.reset();
//    }
//
//    /**
//     * This method will be called repeatedly during the period between when
//     * the START button is pressed and when the OpMode is stopped.
//     */
//    @Override
//    public void loop() {
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Position Error", motor.getPositionError());
//        packet.put("targetPOsition", motorTargetPosition);
//        packet.put("currentPosition", robot.motor.getCurrentPosition());
//        dashboard.sendTelemetryPacket(packet);
//
//
//        double output = motor.calculate(
//                robot.motor.getCurrentPosition(), motorTargetPosition
//        );
//
//            robot.motor.setPower(output);
//
//
//
//
//
//    }
//
//    /**
//     * This method will be called once, when this OpMode is stopped.
//     * <p>
//     * Your ability to control hardware from this method will be limited.
//     */
//    @Override
//    public void stop() {
//
//    }
//}
//
