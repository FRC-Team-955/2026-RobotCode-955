package frc.robot.subsystems.leds;

import frc.robot.BuildConstants;

public class LEDConstants {
    static final int length = 10;

    static final double endgameLowerThresholdSeconds = 25;
    static final double endgameUpperThresholdSeconds = 30;
    static final double lowBatteryThresholdVolts = 12.0;

    static LEDsIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new LEDsIOReal();
            case SIM, REPLAY -> new LEDsIO();
        };
    }
}
