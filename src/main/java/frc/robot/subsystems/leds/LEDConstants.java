package frc.robot.subsystems.leds;

import frc.robot.BuildConstants;

public class LEDConstants {
    static final int length = 10;

    static LEDsIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new LEDsIOroboRIO();
            case SIM, REPLAY -> new LEDsIO();
        };
    }
}
