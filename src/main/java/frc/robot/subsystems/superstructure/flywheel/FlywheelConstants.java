package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.PIDF;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.robot.BuildConstants;

public class FlywheelConstants {
    static final double flywheelRadiusMeters = Units.inchesToMeters(2.0);
    static final double velocityToleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(100);

    static final double gearRatio = 1;
    static final PIDF velocityGains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> PIDF.zero();
        case SIM -> PIDF.ofSV(0.0, 0.02);
    };

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    -1,
                    true,
                    true,
                    40,
                    gearRatio,
                    PIDF.zero(),
                    velocityGains
            );
            case SIM -> new MotorIOSim(
                    gearRatio,
                    0.01,
                    DCMotor.getKrakenX60(2),
                    PIDF.zero(),
                    velocityGains
            );
            case REPLAY -> new MotorIO();
        };
    }
}
