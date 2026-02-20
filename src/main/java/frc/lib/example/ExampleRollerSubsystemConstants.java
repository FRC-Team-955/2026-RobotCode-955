package frc.lib.example;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

public class ExampleRollerSubsystemConstants {
    static final double velocityToleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(10);

    static final double gearRatio = 5;
    static final LoggedTunablePIDF positionGains = switch (BuildConstants.mode) {
        case REAL, REPLAY, SIM -> new LoggedTunablePIDF("ExampleRollerSubsystem/Position");
    };
    static final LoggedTunablePIDF velocityGains = switch (BuildConstants.mode) {
        case REAL, REPLAY, SIM -> new LoggedTunablePIDF("ExampleRollerSubsystem/Velocity");
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
