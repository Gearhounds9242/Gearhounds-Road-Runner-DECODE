/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;
import org.firstinspires.ftc.teamcode.Utilities.VisionSystem;





/*
 * Demonstrates an empty iterative OpMode
 */
@Config
@TeleOp(name = "Mechanum", group = "org/firstinspires/ftc/teamcode/TeleOp")
public class Mechanum extends OpMode {

    private final GearhoundsHardware robot = new GearhoundsHardware();
    private final ElapsedTime runtime = new ElapsedTime();

//    InterpLUT velocityTopLut = new InterpLUT();

    private FtcDashboard dashboard;
    private VisionSystem vision;
    public static double Intake_Speed = 0;
    // back power 1650 for both


    // ========= 自动对齐 & Launch Zone 参数（可在 Dashboard 调） =========
    public static double TARGET_FORWARD_IN = 36.0;   // 你理想的发射距离（离桶多远）
    public static double MIN_FORWARD_IN    = 12.0;   // Launch Zone 前界（离墙太近不对齐）
    public static double MAX_FORWARD_IN    = 72.0;   // Launch Zone 深度（3 tiles = 72in）
    public static double MAX_LATERAL_IN    = 72.0;   // 左右允许范围（6 tiles = 144in，一半72）
    public static double MAX_YAW_DEG       = 25.0;   // 自动对齐允许的最大偏角

    public static double kP_FORWARD = 0.02;
    public static double kP_LATERAL = 0.02;
    public static double kP_YAW     = 0.01;


    //front power 1600 bottom 1480 top
    public static double Top_Speed = 2000;
    public static double Bottom_Speed = 2000;
    public static double shift = 1;

    public static double drop_up = 0.63;
    public static double drop_down = 0.36;
    public static double drop_high = 0.36;
    public static double p2ytime = 0;
    public static int ballNumber = 0;
    public static double twoballtime1 = 0.1;
    public static double twoballtime2 = 0.5;
    public static double twoballtime3 = 1;
    public static double oneballtime1 = 0.1;
    public static double oneballtime2 = 0.5;

    /**
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);

//        velocityTopLut.add(12,1000);
//        velocityTopLut.add(24,2000);
        vision = new VisionSystem(hardwareMap);

    }

    /**
     * This method will be called once, when the START button is pressed.
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * This method will be called repeatedly during the period between when
     * the START button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {

        // Send data to Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("TopMotor", ((robot.TopMotor.getVelocity() / 28) * 60));
        packet.put("BottomMotor", ((robot.BottomMotor.getVelocity() / 28) * 60));
        packet.put("TopVoltage", robot.TopMotor.getCurrent(CurrentUnit.AMPS));
        packet.put("BottomVoltage", robot.BottomMotor.getCurrent(CurrentUnit.AMPS));
        dashboard.sendTelemetryPacket(packet);

        // ========= 1. 从 VisionSystem 读取 Tag 数据 =========
        VisionSystem.TagData tagData = vision.getTagData();

        boolean hasTag = tagData.hasTag;
        boolean inLaunchZone = false;

        double auto_vx = 0.0;    // 前后微调
        double auto_vy = 0.0;    // 左右微调
        double auto_omega = 0.0; // 旋转微调

        if (hasTag) {
            double forward = tagData.forwardIn;
            double lateral = tagData.lateralIn;
            double yawDeg  = tagData.yawDeg;

            // Launch Zone 判定（用 manual 的深度/宽度 + 自己设置的角度容忍）
            inLaunchZone =
                    forward >= MIN_FORWARD_IN &&
                            forward <= MAX_FORWARD_IN &&
                            Math.abs(lateral) <= MAX_LATERAL_IN &&
                            Math.abs(yawDeg)  <= MAX_YAW_DEG;

            telemetry.addData("Tag Forward (in)", forward);
            telemetry.addData("Tag Lateral (in)", lateral);
            telemetry.addData("Tag Yaw (deg)", yawDeg);
            telemetry.addData("In Launch Zone", inLaunchZone);

            if (inLaunchZone) {
                double errorForward = forward - TARGET_FORWARD_IN;  // 正：太远了
                double errorLateral = lateral;                      // 想要 0
                double errorYaw     = yawDeg;                       // 想要 0

                // 注意 forward：往前走会让 forward 距离变小，所以要取负号
                auto_vx    = -errorForward * kP_FORWARD;
                // lateral 假设正值表示目标在右边 → 机器人向右平移
                auto_vy    = errorLateral * kP_LATERAL;
                // yaw 正值时，让机器人往相反方向转
                auto_omega = -errorYaw * kP_YAW;

                telemetry.addData("Auto vx", auto_vx);
                telemetry.addData("Auto vy", auto_vy);
                telemetry.addData("Auto omega", auto_omega);
                telemetry.addData("AutoAlign Status", "ACTIVE");
            } else {
                telemetry.addData("AutoAlign Status", "Tag seen, but not in Launch Zone");
            }
        } else {
            telemetry.addData("AutoAlign Status", "No AprilTag detected");
        }



        if (gamepad1.ps){
            shift = 0.5;
        }
        if (gamepad1.share){
            shift = 1;
        }


        if (gamepad1.right_trigger > 0.1){
            robot.intake.setPower(Intake_Speed);
        } else if (gamepad1.right_trigger <0.99) {
            robot.intake.setPower(0);
        }


        if(gamepad2.right_trigger > 0.1){
            robot.BottomMotor.setPower(Bottom_Speed * gamepad2.right_trigger);
        } else if (gamepad2.right_trigger < 0.99) {
            robot.BottomMotor.setPower(0);
        }

        if(gamepad2.left_trigger > 0.1){
            robot.TopMotor.setPower(Top_Speed * gamepad2.left_trigger);
        } else if (gamepad2.left_trigger < 0.99) {
            robot.TopMotor.setPower(0);
        }


        if (gamepad2.y) {
            p2ytime = runtime.seconds();
        }

        if (((runtime.seconds() - p2ytime) < twoballtime1) && ballNumber == 2) {
            robot.drop.setPosition(0.30);
        } else if (((runtime.seconds() - p2ytime) < twoballtime2) && ballNumber == 2) {
            robot.drop.setPosition(0.69);
        } else if (((runtime.seconds() - p2ytime) < twoballtime3) && ballNumber == 2) {
            robot.drop.setPosition(0.63);
            ballNumber -= 1;
        }

        if (((runtime.seconds() - p2ytime) < oneballtime1) && ballNumber <= 1) {
            robot.drop.setPosition(0.30);
        } else if (((runtime.seconds() - p2ytime) < oneballtime2) && ballNumber <= 1) {
            robot.drop.setPosition(0.63);
            ballNumber = 0;
        }


        if(gamepad2.dpad_down){
            ballNumber -=1;
        }

        if(gamepad2.dpad_up){
            ballNumber +=1;
        }



        if(gamepad2.dpad_right){
            robot.drop.setPosition(drop_high);

        }

        double facing = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        if (gamepad1.options) {
            robot.imu.resetYaw();
        }

        double rotY = y * Math.cos(-facing) - x * Math.sin(-facing);
        double rotX = y * Math.sin(-facing) + x * Math.cos(-facing);
        rotX *= 1.1;

        // ========= 这里把 Driver 控制 + Auto Align 合并 =========
        double vx = rotY;      // 前后：field-centric之后的前进
        double vy = rotX;      // 左右平移
        double omega = rx;     // 旋转

        // 自动对齐是“叠加”，不是替代
        vx    += auto_vx;
        vy    += auto_vy;
        omega += auto_omega;

        // ========= mecanum 轮子功率 =========
        double d = Math.max(Math.abs(vx) + Math.abs(vy) + Math.abs(omega), 1);

        double lf = (vx + vy + omega) / d;
        double lb = (vx - vy + omega) / d;
        double rf = (vx - vy - omega) / d;
        double rb = (vx + vy - omega) / d;

        robot.leftFront.setPower(lf * shift);
        robot.leftBack.setPower(lb * shift);
        robot.rightFront.setPower(rf * shift);
        robot.rightBack.setPower(rb * shift);

        telemetry.addData("", "Intake Speed %f", Intake_Speed);
        telemetry.addData("", "Outtake Top Speed %f", Top_Speed);
        telemetry.addData("", "Outtake Bottom Speed %f", Bottom_Speed);
        telemetry.addData("", "Ball number %d", ballNumber);



    }

}
