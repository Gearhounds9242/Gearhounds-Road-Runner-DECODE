package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Utilities.GearhoundsHardware;


//*****************************************************************

public class Led {
        private final GearhoundsHardware robot;

    public enum Ledcolor {
        OFF(0.0),
        RED(0.279),
        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),
        GREEN(0.5),
        AZURE(0.555),
        BLUE(6.11),
        INDIGO(0.66),
        VIOLET(0.721);

        public final double color;

        Ledcolor(double color) {
            this.color = color;
        }
    }


        public Led(GearhoundsHardware robot) {
            this.robot = robot;
        }

    /**
     * Read here for the color values for the lights <a href="https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2275/15126/3118-0808-0002-Product-Insight-4__88285.1757516465.png?c=1" target="_blank">Example Here</a>
     * @param color  Sets both the lights to the same color
     */

        public Action setColor(double color) {
            return new SetColor(color);
        }

    /**
     * Read here for the color values for the lights <a href="https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2275/15126/3118-0808-0002-Product-Insight-4__88285.1757516465.png?c=1" target="_blank">Example Here</a>
     * @param leftColor  Back left light color 0-1
     * @param rightColor  Back right light color 0-1
     */

        public Action setIndividualColor(double leftColor, double rightColor) {
            return new SetIndividualColor(leftColor, rightColor);
        }

    /**
     * Turns both of the lights off
     */

        public Action Off(){
            return new Off();
        }


        public class SetColor implements Action {
            private final double color;

            public SetColor(double color) {
                this.color = color;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.leftLight.setPosition(color);
                robot.rightLight.setPosition(color);
            return false;
            }
        }






        public class SetIndividualColor implements Action {
            private final double leftColor;
            private final double rightColor;


            public SetIndividualColor(double leftColor, double rightColor) {
                this.leftColor = leftColor;
                this.rightColor = rightColor;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.leftLight.setPosition(leftColor);
                robot.rightLight.setPosition(rightColor);
                return false;
            }
        }


        public class Off implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.leftLight.setPosition(0);
                robot.rightLight.setPosition(0);
                return false;
            }
        }
    }


