package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

import java.util.OptionalDouble;

public class TurretConstants {
    /**
     * Turret positions are robot relative and CCW positive, so 0 means the shooter is aimed
     * exactly where it would be if the turret didn't exist.
     */
    // Exactly one full turn of travel, so every heading is reachable at exactly one position.
    // The cost of that is the boundary: the turret can't cross 180, so following a target past it
    // means sweeping the whole way back around through zero to -180.
    static final double minPositionRad = Units.degreesToRadians(-180.0);
    static final double maxPositionRad = Units.degreesToRadians(180.0);
    /** Only used before homing, and in sim (where we can't hand push the turret into the stops) */
    static final double initialPositionRad = 0.0;

    /**
     * How far apart the two homing points may be from (max - min) before we complain.
     * A mismatch means the gear ratio is wrong.
     */
    static final double homingSpanToleranceRad = Units.degreesToRadians(5.0);

    // PLACEHOLDER: a NEO can't swing a turret 1:1, and the reduction is most of what damps the
    // mechanism, so this needs to be roughly right for the sim to mean anything
    static final double gearRatio = 9.0 * 11.0;

    /** Slower than the turret can physically go, so there's headroom to correct with */
    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(4.0, 12.0);

    /**
     * How far off the two homing points are from the travel we expect between the hard stops.
     * Zero if the gear ratio is right, positive if the encoder thinks the turret moved further
     * than it really did.
     */
    static double homingSpanErrorRad(double rawAtMaxRad, double rawAtMinRad) {
        return (rawAtMaxRad - rawAtMinRad) - (maxPositionRad - minPositionRad);
    }

    /**
     * Position to seed the encoder to, given that we're sitting at the min hard stop. If the two
     * homing points disagree with the expected travel we can't trust either stop over the other,
     * so the error gets split evenly between them.
     */
    static double homingSeedPositionRad(double spanErrorRad) {
        return minPositionRad - spanErrorRad / 2.0;
    }

    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Turret/Gains")
                .withP(2.0)
                .withD(0.1)
                .withS(0.1, StaticFeedforwardSignValue.UseClosedLoopSign);
        case SIM -> new LoggedTunablePIDF("Superstructure/Turret/Gains")
                .withP(8.0)
                .withD(0.1);
    };

    /**
     * Picks the position that aims at the wanted heading without crossing a hard stop, preferring
     * whichever one is closest to where we already are. With exactly a turn of travel that's the
     * heading itself everywhere except right at the boundary, where either stop aims the same way
     * and we stay on the side we're already on.
     *
     * @param wantedRad heading to aim at, wrapped to [-pi, pi]
     * @return the position to drive to, or empty if the turret can't reach the heading at all
     */
    static OptionalDouble reachableSetpointRad(double wantedRad, double currentRad) {
        OptionalDouble best = OptionalDouble.empty();

        // +-1 turn is enough for any turret that doesn't travel more than 540 degrees each way,
        // and leaves this working if the stops ever open up past a single turn
        for (int turns = -1; turns <= 1; turns++) {
            double candidateRad = wantedRad + turns * 2.0 * Math.PI;
            if (candidateRad < minPositionRad || candidateRad > maxPositionRad) continue;

            if (best.isEmpty() || Math.abs(candidateRad - currentRad) < Math.abs(best.getAsDouble() - currentRad)) {
                best = OptionalDouble.of(candidateRad);
            }
        }

        return best;
    }

    static TurretIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new TurretIOSparkMax(
                    99, // PLACEHOLDER: intentionally an invalid CAN ID so it can't drive the wrong device
                    false
            );
            case SIM -> new TurretIOSim(
                    0.3,
                    DCMotor.getNEO(1)
            );
            case REPLAY -> new TurretIO();
        };
    }
}
