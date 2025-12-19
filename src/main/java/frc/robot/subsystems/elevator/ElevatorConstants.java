package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.lib.PIDF;
import frc.robot.BuildConstants;

public class ElevatorConstants {
    /** Gains in radians */
    static final PIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> PIDF.ofPDSVAG(0.1, 0, 0.15, 0.093, 0.005, 0.41);
        case SIM -> PIDF.ofPDVAG(0, 0, 0.11, 0.005, 1.5015);
    };

    static final double maxVelocityMetersPerSecond = 3;
    static final double maxAccelerationMetersPerSecondSquared = 10;

    static final double gearRatio = 5;
    protected static final double sprocketRadiusMeters = Units.inchesToMeters((1.0 + (9.0 / 32.0)) / 2);
    static final double drumRadiusMeters = sprocketRadiusMeters * 3; // 3 stages

    static final double setpointPositionToleranceMeters = Units.inchesToMeters(2);
    static final double setpointVelocityToleranceMetersPerSec = Units.inchesToMeters(2);

    static final double maxHeightMeters = 1.745;
    static final ElevatorLimit upperLimit = new ElevatorLimit(maxHeightMeters - 0.15, 3);
    static final ElevatorLimit lowerLimit = new ElevatorLimit(0.25, -1.75);

    static final double positionOffsetPerMeterOfDistance = 0.9;

    public static final double hardstopMeters = Units.inchesToMeters(13);
    static final double gentleMaxVelocityMetersPerSecond = 0.4;
    /**
     * While we could calculate this based on the current velocity, it caused the gentle profile to be used
     * for only half of the loop cycles. This could probably be solved but I don't think it's worth the effort
     */
    public static double hardstopSlowdownMeters = calculateHardstopSlowdownMeters(maxVelocityMetersPerSecond);

    static double calculateHardstopSlowdownMeters(double currentVelocityMetersPerSec) {
        // In reality, max acceleration is a lot higher, especially with an aggressive kG
        double assumedMaxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared * 0.9;

        // If we are going down at v:
        //     x = -v * t
        // If we slow down at a, max acceleration,
        //     x = -v * t + (1/2) * a * t^2
        // Relating v to the max velocity we want before hitting the hardstop, v_g:
        //     -v_g = -v + a * t
        //     (v - v_g) / a = t
        // Substituting t and doing some simplification:
        //     x = (-v * (v - v_g)) / a + ((v - v_g)^2) / (2 * a)
        // Separating the components to make more readable:
        //     d = (v - v_g)
        double d = currentVelocityMetersPerSec - gentleMaxVelocityMetersPerSecond;
        //     x_v = (-v * d) / a
        double x_v = (-currentVelocityMetersPerSec * d) / assumedMaxAccelerationMetersPerSecondSquared;
        //     x_a = d^2 / (2 * a)
        double x_a = d * d / (2 * assumedMaxAccelerationMetersPerSecondSquared);
        //     x = x_v + x_a
        double x = x_v + x_a;
        // x is negative because we're going down, we want a positive distance above hardstop
        return hardstopMeters + -x;
    }

    static double metersToRad(double meters) {
        return meters / drumRadiusMeters;
    }

    static double radToMeters(double rad) {
        return rad * drumRadiusMeters;
    }

    static ElevatorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new ElevatorIOTalonFX(10, 11, false, false);
            case SIM -> new ElevatorIOSim();
            case REPLAY -> new ElevatorIO();
        };
    }

    record ElevatorLimit(
            double positionMeters,
            double velocityMetersPerSec
    ) {
    }
}