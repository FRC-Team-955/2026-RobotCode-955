package frc.robot.subsystems.superstructure;

import frc.robot.BuildConstants;

public class SuperstructureConstants {
    // https://v6.docs.ctr-electronics.com/en/stable/docs/application-notes/tuning-canrange.html

    static SuperstructureIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new SuperstructureIOCANrange();
            case SIM -> new SuperstructureIOSim();
            case REPLAY -> new SuperstructureIO();
        };
    }
}