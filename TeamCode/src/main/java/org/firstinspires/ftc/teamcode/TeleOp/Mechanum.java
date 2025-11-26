package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;

//import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.VisionSystem;


/**
 * Main TeleOp for field-centric mecanum drive + intake + shooter + drop.
 * Also includes AprilTag auto-alignment:
 *   - Only adjusts lateral (strafe) and heading (yaw)
 *   - Does NOT change forward/back distance
 *   - Only aligns to your own goal (filter by tag ID + alliance flag)
 */
@Config
@TeleOp(name = "Mechanum", group = "TeamCode/TeleOp")
public class Mechanum extends OpMode {
    private final GearhoundsHardware robot = new GearhoundsHardware();
    private final ElapsedTime runtime = new ElapsedTime();
//     InterpLUT velocityTopLut = new InterpLUT();
//     InterpLUT velocityBottomLut = new InterpLUT();
    private FtcDashboard dashboard;
    private VisionSystem vision;

    public static boolean isRedAlliance = false;   // set false when you are blue
    private MecanumDrive drive;
    public static double Intake_Speed = 0.0;
    // back power 1650 for both
    // front power 1600 bottom 1480 top
    public static double Top_Speed = 2000.0;
    public static double Bottom_Speed = 2000.0;
    public static double shift = 1.0;
    // Drop servo + ball count logic
    public static double drop_up = 0.63;
    public static double drop_down = 0.36;
    public static double drop_high = 0.36;

    public static double p2ytime = 0.0;
    public static int ballNumber = 0;

    // two-ball timing (seconds)
    public static double twoballtime1 = 0.1;
    public static double twoballtime2 = 0.5;
    public static double twoballtime3 = 1.0;

    // one-ball timing (seconds)
    public static double oneballtime1 = 0.1;
    public static double oneballtime2 = 0.5;

    // -----------------------------
    // Auto-align parameters
    // -----------------------------
    // We do NOT control forward distance; driver decides how far to stand.
    // These bounds are only used to reject obviously bad distances.
    public static double MIN_FORWARD_IN = 12.0;   // too close to the goal? don't auto-align
    public static double MAX_FORWARD_IN = 110.0;  // too far from goal? don't auto-align

    // Maximum yaw magnitude (deg) where we still try to auto-align
    public static double MAX_YAW_DEG = 25.0;

    // P gains for auto-correction
    public static double kP_LATERAL = 0.02; // lateral correction
    public static double kP_YAW     = 0.01; // heading correction

    // Alliance side:
    // true  = we are targeting the RED goal
    // false = we are targeting the BLUE goal
    public static boolean USE_RED_GOAL = true;

    // Tag IDs for your OWN goal. You MUST fill these with real IDs from the DECODE manual.
    // Current numbers are placeholders, change them to correct ones.
    public static int[] RED_GOAL_TAG_IDS  = { 11, 12, 13 }; // TODO: replace with actual red goal tag IDs
    public static int[] BLUE_GOAL_TAG_IDS = { 21, 22, 23 }; // TODO: replace with actual blue goal tag IDs

    /**
     * Returns true only if this AprilTag ID belongs to our goal on this alliance.
     * This prevents auto-aligning to the opponent's goal.
     */
    private boolean isOurGoalTag(int id) {
        int[] arr = USE_RED_GOAL ? RED_GOAL_TAG_IDS : BLUE_GOAL_TAG_IDS;
        for (int t : arr) {
            if (t == id) return true;
        }
        return false;
    }

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

//         velocityTopLut.add(72, 1480);
//         velocityTopLut.add(48,1180);
//         velocityTopLut.add(108.5,1650);
//         velocityTopLut.add(96, 1550);
//         velocityTopLut.createLUT();
//
//         velocityBottomLut.add(72, 1540);
//         velocityBottomLut.add(48,2000);
//         velocityBottomLut.add(108.5,1650);
//         velocityBottomLut.add(96, 1550);
//         velocityBottomLut.createLUT();

        robot.init(hardwareMap);             // motors/servos/IMU setup
        vision = new VisionSystem(hardwareMap); // webcam + AprilTag setup
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // -----------------------------
    // START (called once when you hit PLAY)
    // -----------------------------
    @Override
    public void start() {
        runtime.reset();
    }

    // -----------------------------
    // LOOP (runs repeatedly while opmode is active)
    // -----------------------------
    @Override
    public void loop() {

        // -------------------------------------------------------
        // 1) Dashboard shooter telemetry
        // -------------------------------------------------------
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("TopMotorRPM",    (robot.TopMotor.getVelocity()    / 28.0) * 60.0);
        packet.put("BottomMotorRPM", (robot.BottomMotor.getVelocity() / 28.0) * 60.0);
        packet.put("TopCurrentA",    robot.TopMotor.getCurrent(CurrentUnit.AMPS));
        packet.put("BottomCurrentA", robot.BottomMotor.getCurrent(CurrentUnit.AMPS));
        dashboard.sendTelemetryPacket(packet);

        // -------------------------------------------------------
        // 2) Driver speed shift (gamepad1 PS/Share)
        // -------------------------------------------------------
        if (gamepad1.ps)    shift = 0.5; // slow mode
        if (gamepad1.share) shift = 1.0; // full speed

        // -------------------------------------------------------
        // 3) Intake control (gamepad1 right trigger)
        // -------------------------------------------------------
        if (gamepad1.right_trigger > 0.1) {
            robot.intake.setPower(Intake_Speed);
        } else {
            robot.intake.setPower(0.0);
        }

        // -------------------------------------------------------
        // 4) Shooter bottom motor (gamepad2 right trigger)
        // -------------------------------------------------------
        if (gamepad2.right_trigger > 0.1) {
            robot.BottomMotor.setPower(Bottom_Speed * gamepad2.right_trigger);
        } else {
            robot.BottomMotor.setPower(0.0);
        }

        // -------------------------------------------------------
        // 5) Shooter top motor (gamepad2 left trigger)
        // -------------------------------------------------------
        if (gamepad2.left_trigger > 0.1) {
            robot.TopMotor.setPower(Top_Speed * gamepad2.left_trigger);
        } else {
            robot.TopMotor.setPower(0.0);
        }

        // -------------------------------------------------------
        // 6) Drop servo timing logic (unchanged from your original)
        // -------------------------------------------------------
        if (gamepad2.y) {
            // Record time when Y is pressed to start a drop cycle
            p2ytime = runtime.seconds();
        }

        // Two-ball sequence
        if (((runtime.seconds() - p2ytime) < twoballtime1) && ballNumber == 2) {
            robot.drop.setPosition(0.28);
        } else if (((runtime.seconds() - p2ytime) < twoballtime2) && ballNumber == 2) {
            robot.drop.setPosition(0.69);
        } else if (((runtime.seconds() - p2ytime) < twoballtime3) && ballNumber == 2) {
            robot.drop.setPosition(0.63);
            ballNumber -= 1;
        }

        // One-ball sequence
        if (((runtime.seconds() - p2ytime) < oneballtime1) && ballNumber <= 1) {
            robot.drop.setPosition(0.30);
        } else if (((runtime.seconds() - p2ytime) < oneballtime2) && ballNumber <= 1) {
            robot.drop.setPosition(0.63);
            ballNumber = 0;
        }

        // Manual ball count adjustments
        if (gamepad2.dpad_down) ballNumber--;
        if (gamepad2.dpad_up)   ballNumber++;

        // Manual drop override
        if (gamepad2.dpad_right) {
            robot.drop.setPosition(drop_high);
        }

        // -------------------------------------------------------
        // 7) AprilTag auto-align: read tag data and compute corrections
        // -------------------------------------------------------
        VisionSystem.TagData tagData = vision.getTagData();

        boolean hasTag = tagData.hasTag;
        boolean inLaunchZone = false;

        // Auto-alignment velocity components (robot-centric)
        double auto_vx = 0.0;    // forward/back (we keep this ZERO: driver controls distance)
        double auto_vy = 0.0;    // strafe left/right
        double auto_omega = 0.0; // rotation

        if (hasTag) {
            telemetry.addData("Tag ID", tagData.id);

            // First, check if this tag belongs to our own goal (RED or BLUE).
            if (isOurGoalTag(tagData.id)) {
                double forward = tagData.forwardIn;
                double lateral = tagData.lateralIn;
                double yawDeg  = tagData.yawDeg;

                // Simple "launch zone" check:
                // - not too close
                // - not too far
                // - yaw not insane
                inLaunchZone =
                        forward >= MIN_FORWARD_IN &&
                                forward <= MAX_FORWARD_IN &&
                                Math.abs(yawDeg) <= MAX_YAW_DEG;

                telemetry.addData("Tag Forward (in)", forward);
                telemetry.addData("Tag Lateral (in)", lateral);
                telemetry.addData("Tag Yaw (deg)", yawDeg);
                telemetry.addData("In Launch Range", inLaunchZone);

                if (inLaunchZone) {
                    // We only correct lateral and yaw.
                    double errorLateral = lateral; // want to drive this toward 0
                    double errorYaw     = yawDeg;  // want to drive this toward 0

                    auto_vx    = 0.0;                           // driver owns forward/back
                    auto_vy    = errorLateral * kP_LATERAL;     // strafe
                    auto_omega = -errorYaw * kP_YAW;            // rotate opposite the yaw error

                    telemetry.addData("Auto vx", auto_vx);
                    telemetry.addData("Auto vy", auto_vy);
                    telemetry.addData("Auto omega", auto_omega);
                    telemetry.addData("AutoAlign Status", "ACTIVE (lat + yaw on own goal)");
                } else {
                    telemetry.addData("AutoAlign Status",
                            "Tag is our goal but not in launch range (no auto-align)");
                }
            } else {
                // Tag is NOT one of our goal tags: ignore it for auto-align
                telemetry.addData("AutoAlign Status",
                        "Tag is NOT our goal (ignored for auto-align)");
            }
        } else {
            telemetry.addData("AutoAlign Status", "No AprilTag detected");
        }

        // -------------------------------------------------------
        // 8) Field-centric drive + auto-align overlay
        // -------------------------------------------------------

        // IMU yaw in radians for field-centric transformation
        double facing = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        Pose2d pose = drive.localizer.getPose();
        double heading = pose.heading.toDouble(); // radians

        drive.setDrivePowers(drive.localizer.getPose().heading.inverse().times(new PoseVelocity2d(
                new Vector2d(
                        (-gamepad1.left_stick_y * shift),
                        (-gamepad1.left_stick_x * shift)
                ),
                (-gamepad1.right_stick_x * shift)
        )));


        if (gamepad1.optionsWasPressed()) {
            drive.localizer.setPose(
                    new Pose2d(
                            drive.localizer.getPose().position,
                            0.0
                    )
            );
        }

        // Update Road Runner’s localization
        drive.localizer.update();


        // -------------------------------------------------------
        // 9) Telemetry to Driver Station
        // -------------------------------------------------------
        telemetry.addData("Intake Power", Intake_Speed);
        telemetry.addData("Top Shooter Scale", Top_Speed);
        telemetry.addData("Bottom Shooter Scale", Bottom_Speed);
        telemetry.addData("Ball Count", ballNumber);
        telemetry.addData("Use Red Goal", USE_RED_GOAL);
        telemetry.update();
    }
}
