package frc.robot.subsystems.superintake.intakepivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

public class IntakePivotConstants {
    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 3);

    static final double minPositionRad = Units.degreesToRadians(16.827716);
    static final double maxPositionRad = Units.degreesToRadians(95.554559);
    static final double initialPositionRad = minPositionRad;

    static final double gearRatio = 150;

    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superintake/IntakePivot/Gains");
        case SIM ->
                new LoggedTunablePIDF("Superintake/IntakePivot/Gains").withP(20.0).withG(2.65, GravityTypeValue.Arm_Cosine);
    };

    static IntakePivotIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new IntakePivotIOTalonFX(
                    -1,
                    true
            );
            case SIM -> new IntakePivotIOSim(
                    0.0768892879,
                    DCMotor.getKrakenX60(1)
            );
            case REPLAY -> new IntakePivotIO();
        };
    }
}
