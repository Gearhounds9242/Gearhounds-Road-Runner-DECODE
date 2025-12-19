package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Leds {

    // ===== SOLID COLORS（常亮）=====
    public static final RevBlinkinLedDriver.BlinkinPattern SOLID_RED =
            RevBlinkinLedDriver.BlinkinPattern.RED;

    public static final RevBlinkinLedDriver.BlinkinPattern SOLID_BLUE =
            RevBlinkinLedDriver.BlinkinPattern.BLUE;

    public static final RevBlinkinLedDriver.BlinkinPattern SOLID_GREEN =
            RevBlinkinLedDriver.BlinkinPattern.GREEN;

    public static final RevBlinkinLedDriver.BlinkinPattern SOLID_YELLOW =
            RevBlinkinLedDriver.BlinkinPattern.YELLOW;

    public static final RevBlinkinLedDriver.BlinkinPattern SOLID_ORANGE =
            RevBlinkinLedDriver.BlinkinPattern.ORANGE;

    public static final RevBlinkinLedDriver.BlinkinPattern SOLID_PURPLE =
            RevBlinkinLedDriver.BlinkinPattern.VIOLET;

    public static final RevBlinkinLedDriver.BlinkinPattern SOLID_WHITE =
            RevBlinkinLedDriver.BlinkinPattern.WHITE;


    // ===== BREATH（呼吸灯）=====
    public static final RevBlinkinLedDriver.BlinkinPattern BREATH_RED =
            RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;

    public static final RevBlinkinLedDriver.BlinkinPattern BREATH_BLUE =
            RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;

    // ===== STROBE（闪烁 / 报警）=====
    public static final RevBlinkinLedDriver.BlinkinPattern STROBE_RED =
            RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;

    public static final RevBlinkinLedDriver.BlinkinPattern STROBE_BLUE =
            RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE;

    public static final RevBlinkinLedDriver.BlinkinPattern STROBE_GOLD =
            RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;


    // ===== HEARTBEAT（慢闪）=====
    public static final RevBlinkinLedDriver.BlinkinPattern HEARTBEAT_RED =
            RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;

    public static final RevBlinkinLedDriver.BlinkinPattern HEARTBEAT_BLUE =
            RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;


    // ===== FUN / SPECIAL =====
    public static final RevBlinkinLedDriver.BlinkinPattern CONFETTI =
            RevBlinkinLedDriver.BlinkinPattern.CONFETTI;

    public static final RevBlinkinLedDriver.BlinkinPattern RAINBOW =
            RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

    public static final RevBlinkinLedDriver.BlinkinPattern PARTY =
            RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE;
}

/* =========================================================
 * HOW TO USE LEDS
 * =========================================================
 *
 *   leds.set(Leds.XXXXXXX);

 * EXAMPLES:
 *
 *   if (lowBattery) {
 *       leds.set(Leds.STROBE_RED);
 *   }
 *   else if (endgame) {
 *       leds.set(Leds.RAINBOW);
 *   }
 *   else if (shooterReady) {
 *       leds.set(Leds.BREATH_GREEN);
 *   }
 *   else {
 *       leds.set(Leds.SOLID_BLUE);
 *   }
 *
 * =========================================================
 */