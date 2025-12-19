package frc.lib.motor;

import edu.wpi.first.math.util.Units;

public record RequestTolerances(
        double positionToleranceRad,
        double velocityToleranceRadPerSec,
        double voltageToleranceVolts
) {
    private static final double defaultPositionToleranceRad = Units.degreesToRadians(15);
    private static final double defaultVelocityToleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(30);
    private static final double defaultVoltageToleranceVolts = 1.0;

    public static RequestTolerances defaults() {
        return new RequestTolerances(defaultPositionToleranceRad, defaultVelocityToleranceRadPerSec, defaultVoltageToleranceVolts);
    }

    public static RequestTolerances position(double positionToleranceRad) {
        return new RequestTolerances(positionToleranceRad, defaultVelocityToleranceRadPerSec, defaultVoltageToleranceVolts);
    }

    public static RequestTolerances velocity(double velocityToleranceRadPerSec) {
        return new RequestTolerances(defaultPositionToleranceRad, velocityToleranceRadPerSec, defaultVoltageToleranceVolts);
    }

    public static RequestTolerances voltage(double voltageToleranceVolts) {
        return new RequestTolerances(defaultPositionToleranceRad, defaultVelocityToleranceRadPerSec, voltageToleranceVolts);
    }
}
