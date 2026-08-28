package frc.robot.subsystems.superstructure.turret;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

public class TurretConstants {
    static final double positionToleranceRad = Units.degreesToRadians(10);

    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 3);

    // 0 = turret pointing straight forward on the chassis
    static final double minPositionRad = Units.degreesToRadians(-180);
    static final double maxPositionRad = Units.degreesToRadians(180);
    static final double initialPositionRad = 0;

    // TODO: CHANGE THIS
    static final double gearRatio = 3.0 * 3.0 * (68.0 / 12.0);
    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Turret/Gains")
                .withP(0.5)
                .withD(0.0);
        case SIM -> new LoggedTunablePIDF("Superstructure/Turret/Gains")
                .withP(30.0)
                .withD(0.0);
    };

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    11,
                    true,
                    SparkBaseConfig.IdleMode.kBrake,
                    40,
                    gearRatio,
                    gains,
                    null,
                    initialPositionRad
            );
            // NOTE: if you are doing an arm, use MotorIOArmSim
            case SIM -> new MotorIOSim(
                    gearRatio,
                    0.01,
                    DCMotor.getNEO(1),
                    gains,
                    null,
                    initialPositionRad
            );
            case REPLAY -> new MotorIO();
        };
    }
}
