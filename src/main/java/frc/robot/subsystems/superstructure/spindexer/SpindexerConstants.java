package frc.robot.subsystems.superstructure.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.PIDF;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.robot.BuildConstants;

public class SpindexerConstants {
    static final double gearRatio = 5;
    static final PIDF positionGains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> PIDF.zero();
        case SIM -> PIDF.zero();
    };
    static final PIDF velocityGains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> PIDF.zero();
        case SIM -> PIDF.zero();
    };

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    -1,
                    true,
                    true,
                    40,
                    gearRatio,
                    positionGains,
                    velocityGains
            );
            case SIM -> new MotorIOSim(
                    gearRatio,
                    0.01,
                    DCMotor.getNEO(1),
                    positionGains,
                    velocityGains
            );
            case REPLAY -> new MotorIO();
        };
    }
}
