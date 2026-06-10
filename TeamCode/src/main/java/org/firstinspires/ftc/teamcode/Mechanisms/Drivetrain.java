package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Drivetrain {

    // Tunable constants
    public static double HEADING_TOLERANCE = Math.toRadians(2.0); // 2 degrees
    public static double TURN_GAIN         = 2;                 // proportional gain
    public static double MAX_TURN_POWER    = 0.6;                 // cap turn speed
    public static double MIN_TURN_POWER    = 0.1;                 // prevent stalling
    public static double STABLE_COUNT_REQ  = 10;                  // loops within tolerance before done

    private final MecanumDrive drive;

    public Drivetrain(MecanumDrive drive) {
        this.drive = drive;
    }

    /**
     * Returns an Action that turns the robot to face a field coordinate.
     *
     * @param targetX  X coordinate on the field to face
     * @param targetY  Y coordinate on the field to face
     */
    public Action turnTo(double targetX, double targetY) {
        return new TurnTo(targetX, targetY, 0);
    }

    private class TurnTo implements Action {
        private final double targetX;
        private final double targetY;
        private final double offset;
        private int stableCount = 0;

        TurnTo(double targetX, double targetY, double offset) {
            this.targetX = targetX;
            this.targetY = targetY;
            this.offset  = offset;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            drive.localizer.update();

            Pose2d current = drive.localizer.getPose();

            // Calculate the angle from robot to target point
            double dx = targetX - current.position.x;
            double dy = targetY - current.position.y;
            double angleToTarget = Math.atan2(dy, dx) + offset;

            // Find shortest path to that angle from current heading
            double currentHeading = current.heading.toDouble();
            double headingError   = angleToTarget - currentHeading;

            // Normalize to -PI to PI so we always take the short way around
            while (headingError >  Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            packet.put("TurnToPoint headingError (deg)", Math.toDegrees(headingError));
            packet.put("TurnToPoint targetAngle (deg)",  Math.toDegrees(angleToTarget));

            // Check if we are within tolerance
            if (Math.abs(headingError) < HEADING_TOLERANCE) {
                stableCount++;
                if (stableCount >= STABLE_COUNT_REQ) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    return false; // done
                }
            } else {
                stableCount = 0; // reset if we drift out
            }

            // Proportional turn, clamped to min/max
            double turnPower = headingError * TURN_GAIN;
            turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));

            // Prevent motor stall at very small errors
            if (Math.abs(turnPower) < MIN_TURN_POWER && Math.abs(headingError) > HEADING_TOLERANCE) {
                turnPower = Math.copySign(MIN_TURN_POWER, turnPower);
            }

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
            return true;
        }
    }
}