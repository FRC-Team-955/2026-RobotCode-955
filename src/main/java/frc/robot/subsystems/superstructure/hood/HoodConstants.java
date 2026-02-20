package frc.robot.subsystems.superstructure.hood;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

public class HoodConstants {
    static final double positionToleranceRad = Units.degreesToRadians(10);

    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 3);

    static final double gearRatio = 120;
    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Hood/Gains");
        case SIM -> new LoggedTunablePIDF("Superstructure/Hood/Gains").withP(18.9);
    };

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    -1,
                    true,
                    SparkBaseConfig.IdleMode.kBrake,
                    40,
                    gearRatio,
                    gains,
                    null
            );
            case SIM -> new MotorIOSim(
                    gearRatio,
                    0.01,
                    DCMotor.getNEO(1),
                    gains,
                    null
            );
            case REPLAY -> new MotorIO();
        };
    }
}
