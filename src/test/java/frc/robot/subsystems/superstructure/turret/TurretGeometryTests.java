package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

import static frc.robot.subsystems.superstructure.turret.TurretConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

class TurretGeometryTests {
    private static final double epsilon = 1e-9;

    /** What the max stop reads after homing seeds the encoder at the min stop */
    private static double impliedMaxPositionRad(double angleAtMaxRad, double angleAtMinRad) {
        return homingSeedPositionRad(homingSpanErrorRad(angleAtMaxRad, angleAtMinRad))
                + (angleAtMaxRad - angleAtMinRad);
    }

    @Test
    void perfectGearRatioLandsOnBothStops() {
        double angleAtMin = 10.0;
        double angleAtMax = angleAtMin + (maxPositionRad - minPositionRad);

        assertEquals(0.0, homingSpanErrorRad(angleAtMax, angleAtMin), epsilon);
        assertEquals(minPositionRad, homingSeedPositionRad(homingSpanErrorRad(angleAtMax, angleAtMin)), epsilon);
        assertEquals(maxPositionRad, impliedMaxPositionRad(angleAtMax, angleAtMin), epsilon);
    }

    @Test
    void badGearRatioSplitsTheErrorBetweenTheStops() {
        double span = maxPositionRad - minPositionRad;
        // Encoder overreports travel by 10%
        double measuredSpan = span * 1.1;
        double expectedError = span * 0.1;

        assertEquals(expectedError, homingSpanErrorRad(measuredSpan, 0.0), epsilon);
        assertEquals(
                minPositionRad - expectedError / 2.0,
                homingSeedPositionRad(homingSpanErrorRad(measuredSpan, 0.0)),
                epsilon
        );
        assertEquals(
                maxPositionRad + expectedError / 2.0,
                impliedMaxPositionRad(measuredSpan, 0.0),
                epsilon
        );
    }

    @Test
    void staysOnTheSideItIsAlreadyOnAtTheBoundary() {
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
        // 181 degrees doesn't exist, so tracking past the stop has to come back the other way
        assertEquals(
                Units.degreesToRadians(-179.0),
                reachableSetpointRad(Units.degreesToRadians(-179.0), Units.degreesToRadians(179.0)).orElseThrow(),
                epsilon
        );
    }
}
