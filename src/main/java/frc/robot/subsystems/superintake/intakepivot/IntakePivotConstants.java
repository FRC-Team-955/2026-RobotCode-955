package frc.robot.subsystems.superintake.intakepivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOArmSim;
import frc.lib.motor.MotorIOTalonFX;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

public class IntakePivotConstants {
    static final double positionToleranceRad = Units.degreesToRadians(10);

    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 3);

    static final double gearRatio = 150;
    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superintake/IntakePivot/Gains");
        case SIM ->
                new LoggedTunablePIDF("Superintake/IntakePivot/Gains").withP(20.0).withG(2.65, GravityTypeValue.Arm_Cosine);
    };

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOTalonFX(
                    -1,
                    true,
                    NeutralModeValue.Coast,
                    40,
                    gearRatio,
                    gains,
                    null
            );
            case SIM -> new MotorIOArmSim(
                    DCMotor.getNEO(1),
                    gearRatio,
                    0.0768892879,
                    Units.inchesToMeters(10),
                    Units.degreesToRadians(-90),
                    Units.degreesToRadians(0),
                    true,
                    Units.degreesToRadians(0),
                    0.001,
                    gains
            );
            case REPLAY -> new MotorIO();
        };
    }
}
