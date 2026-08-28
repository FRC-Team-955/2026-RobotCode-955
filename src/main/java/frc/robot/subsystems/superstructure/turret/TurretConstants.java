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

public class TurretConstants {
    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);

    static final double minPositionRad = Units.degreesToRadians(16.827716);
    static final double maxPositionRad = Units.degreesToRadians(95.554559);
    static final double initialPositionRad = maxPositionRad;

    static final double gearRatio = 3.0 * 3.0 * (68.0 / 12.0);

    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superintake/Turret/Gains")
                .withP(0.0)
                .withS(0.0, StaticFeedforwardSignValue.UseClosedLoopSign);
        case SIM -> new LoggedTunablePIDF("Superintake/Turret/Gains")
                .withP(20.0);
    };

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    11,
                    false,
                    SparkBaseConfig.IdleMode.kBrake,
                    40,
                    gearRatio,
                    gains,
                    null,
                    initialPositionRad
            );
            case SIM -> new MotorIOArmSim(
                    DCMotor.getNEO(1),
                    gearRatio,
                    0.1,
                    Units.inchesToMeters(10),
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
