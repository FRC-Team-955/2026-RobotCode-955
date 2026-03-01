package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

public class HoodConstants {
    static final double minPositionRad = Units.degreesToRadians(15.0);
    static final double maxPositionRad = Units.degreesToRadians(45.0);
    static final double initialPositionRad = minPositionRad;
    static final double maxPositionUnderTrench = Units.degreesToRadians(30.0);

    /**
     * Hood angle for vertical shot is 0°
     * Shooting angle for vertical shot is 90°
     * Shooting angle for 15° from vertical is 75°
     * Therefore, hood angle = 90° - shooting angle
     * and shooting angle = 90° - hood angle
     */
    public static double convertBetweenShotAngleAndHoodAngleRad(double originalAngleRad) {
        return Math.PI / 2.0 - originalAngleRad;
    }

    static final double gearRatio = 6.0 * (220.0 / 20.0);
    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Hood/Gains")
                .withP(5)
                .withG(0, GravityTypeValue.Arm_Cosine);
        case SIM -> new LoggedTunablePIDF("Superstructure/Hood/Gains")
                .withP(30)
                .withG(0.3, GravityTypeValue.Arm_Cosine);
    };

    static HoodIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new HoodIOSparkMax(
                    10,
                    true
            );
            case SIM -> new HoodIOSim(
                    0.01,
                    DCMotor.getNeo550(1)
            );
            case REPLAY -> new HoodIO();
        };
    }
}
