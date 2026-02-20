package frc.robot.subsystems.superstructure.flywheel;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

public class FlywheelConstants {
    public static final double flywheelRadiusMeters = Units.inchesToMeters(2.0);
    static final double velocityToleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(100);

    static final double gearRatio = 1;
    static final LoggedTunablePIDF velocityGains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Flywheel/Gains");
        case SIM -> new LoggedTunablePIDF("Superstructure/Flywheel/Gains").withV(0.02);
    };

    static FlywheelIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new FlywheelIOTalonFX(
                    -1,
                    -1,
                    false,
                    MotorAlignmentValue.Opposed
            );
            case SIM -> new FlywheelIOSim(
                    0.01,
                    DCMotor.getKrakenX60(2)
            );
            case REPLAY -> new FlywheelIO();
        };
    }
}
