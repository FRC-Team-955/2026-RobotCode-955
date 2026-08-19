package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

import static frc.robot.subsystems.superstructure.turret.TurretConstants.*;
import static org.junit.jupiter.api.Assertions.*;

class TurretGeometryTests {
    private static final double epsilon = 1e-9;

    /** Where the max stop ends up reading after homing seeds the encoder at the min stop */
    private static double impliedMaxPositionRad(double rawAtMaxRad, double rawAtMinRad) {
        return homingSeedPositionRad(homingSpanErrorRad(rawAtMaxRad, rawAtMinRad)) + (rawAtMaxRad - rawAtMinRad);
    }

    @Test
    void perfectGearRatioLandsOnBothStops() {
        // Encoder happened to be reading 10 rad when we pushed into the min stop
        double rawAtMin = 10.0;
        double rawAtMax = rawAtMin + (maxPositionRad - minPositionRad);

        assertEquals(0.0, homingSpanErrorRad(rawAtMax, rawAtMin), epsilon);
        assertEquals(minPositionRad, homingSeedPositionRad(homingSpanErrorRad(rawAtMax, rawAtMin)), epsilon);
        assertEquals(maxPositionRad, impliedMaxPositionRad(rawAtMax, rawAtMin), epsilon);
    }

    @Test
    void onlyTheDistanceBetweenTheStopsMatters() {
        double span = maxPositionRad - minPositionRad;

        assertEquals(
                homingSeedPositionRad(homingSpanErrorRad(span, 0.0)),
                homingSeedPositionRad(homingSpanErrorRad(-500.0 + span, -500.0)),
                epsilon
        );
    }

    @Test
    void badGearRatioSplitsTheErrorBetweenTheStops() {
        double span = maxPositionRad - minPositionRad;
        // Gear ratio is 10% too small, so the encoder overreports travel
        double measuredSpan = span * 1.1;
        double expectedError = span * 0.1;

        assertEquals(expectedError, homingSpanErrorRad(measuredSpan, 0.0), epsilon);
        assertEquals(
                minPositionRad - expectedError / 2.0,
                homingSeedPositionRad(homingSpanErrorRad(measuredSpan, 0.0)),
                epsilon
        );
        // Both ends are off by the same amount in opposite directions, instead of one end eating all of it
        assertEquals(
                maxPositionRad + expectedError / 2.0,
                impliedMaxPositionRad(measuredSpan, 0.0),
                epsilon
        );
    }

    @Test
    void staysOnTheSideItIsAlreadyOnAtTheBoundary() {
        // 180 and -180 aim at the same place, and with a full turn of travel both are in range,
        // so don't cross the whole turret to reach the far one
        assertEquals(
                maxPositionRad,
                reachableSetpointRad(Math.PI, Units.degreesToRadians(170.0)).orElseThrow(),
                epsilon
        );
        assertEquals(
                minPositionRad,
                reachableSetpointRad(-Math.PI, Units.degreesToRadians(-170.0)).orElseThrow(),
                epsilon
        );
    }

    @Test
    void sweepsBackAroundInsteadOfCrossingTheStop() {
        // Turret is nearly at the CCW stop and the robot rotates the target past it. Continuing
        // would need 181 degrees, which doesn't exist, so it has to come back the other way.
        double setpointRad = reachableSetpointRad(
                Units.degreesToRadians(-179.0),
                Units.degreesToRadians(179.0)
        ).orElseThrow();

        assertEquals(Units.degreesToRadians(-179.0), setpointRad, epsilon);
        assertTrue(setpointRad >= minPositionRad && setpointRad <= maxPositionRad);
    }

    @Test
    void everyHeadingIsReachable() {
        // A full turn of travel is what makes the out of range alert dead code - narrow the stops
        // and it starts firing
        assertTrue(maxPositionRad - minPositionRad >= 2.0 * Math.PI);

        for (double degrees = -180.0; degrees <= 180.0; degrees += 5.0) {
            double wanted = Units.degreesToRadians(degrees);
            assertTrue(
                    reachableSetpointRad(wanted, 0.0).isPresent(),
                    "turret should be able to reach " + degrees + " degrees"
            );
        }
    }
}
