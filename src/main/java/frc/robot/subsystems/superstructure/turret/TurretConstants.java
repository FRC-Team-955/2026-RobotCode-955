package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOArmSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

import java.util.OptionalDouble;

public class TurretConstants {
    /** Robot relative and CCW positive, so 0 aims the shooter where a fixed shooter would point */
    static final double minPositionRad = Units.degreesToRadians(-180.0);
    static final double maxPositionRad = Units.degreesToRadians(180.0);
    /** Only used before homing, and in sim, where nobody can push the turret into a stop */
    static final double initialPositionRad = 0.0;

    /** Homing points further apart than this from (max - min) mean the gear ratio is wrong */
    static final double homingSpanToleranceRad = Units.degreesToRadians(5.0);

    static final double gearRatio = 9.0 * 11.0; // PLACEHOLDER

    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(4.0, 12.0);

    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Turret/Gains")
                .withP(2.0)
                .withD(0.1)
                .withS(0.1, StaticFeedforwardSignValue.UseClosedLoopSign);
        case SIM -> new LoggedTunablePIDF("Superstructure/Turret/Gains")
                .withP(8.0)
                .withD(0.1);
    };

    /** How far off the two homing points are from the travel we expect between the hard stops */
    static double homingSpanErrorRad(double angleAtMaxRad, double angleAtMinRad) {
        return (angleAtMaxRad - angleAtMinRad) - (maxPositionRad - minPositionRad);
    }

    /**
     * Position to seed the encoder to, given that we're sitting at the min hard stop. The error
     * is split evenly between the stops because neither one is more trustworthy than the other.
     */
    static double homingSeedPositionRad(double spanErrorRad) {
        return minPositionRad - spanErrorRad / 2.0;
    }

    /**
     * Position that aims at a wanted angle without crossing a hard stop, preferring the one
     * closest to where we already are. Empty if the turret can't reach the angle at all.
     *
     * @param wantedAngleRad robot relative, wrapped to [-pi, pi]
     */
    static OptionalDouble reachableSetpointRad(double wantedAngleRad, double currentAngleRad) {
        OptionalDouble best = OptionalDouble.empty();

        for (int turns = -1; turns <= 1; turns++) {
            double candidateRad = wantedAngleRad + turns * 2.0 * Math.PI;
            if (candidateRad < minPositionRad || candidateRad > maxPositionRad) continue;

            if (best.isEmpty() || Math.abs(candidateRad - currentAngleRad) < Math.abs(best.getAsDouble() - currentAngleRad)) {
                best = OptionalDouble.of(candidateRad);
            }
        }

        return best;
    }

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    99, // PLACEHOLDER: invalid on purpose so it can't drive the wrong device
                    false,
                    SparkBaseConfig.IdleMode.kBrake,
                    30,
                    gearRatio,
                    gains,
                    null,
                    initialPositionRad
            );
            // An arm with gravity off is a rotating platform, and it stops at the hard stops
            case SIM -> new MotorIOArmSim(
                    DCMotor.getNEO(1),
                    gearRatio,
                    0.3,
                    0.3,
                    minPositionRad,
                    maxPositionRad,
                    false,
                    initialPositionRad,
                    0.001,
                    gains
            );
            case REPLAY -> new MotorIO();
        };
    }
}
