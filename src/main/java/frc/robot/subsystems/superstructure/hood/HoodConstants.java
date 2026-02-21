package frc.robot.subsystems.superstructure.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

public class HoodConstants {
    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 3);

    static final double minPositionRad = Units.degreesToRadians(15.0);
    static final double maxPositionRad = Units.degreesToRadians(45.0);
    static final double initialPositionRad = minPositionRad;
    static final double maxPositionUnderTrench = Units.degreesToRadians(35.0);

    static final double gearRatio = 120;
    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Hood/Gains");
        case SIM -> new LoggedTunablePIDF("Superstructure/Hood/Gains").withP(18.9);
    };

    static HoodIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new HoodIOSparkMax(
                    -1,
                    false
            );
            case SIM -> new HoodIOSim(
                    0.01,
                    DCMotor.getNeo550(1)
            );
            case REPLAY -> new HoodIO();
        };
    }
}
