package frc.lib.example;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

public class ExampleServoSubsystemConstants {
    static final double positionToleranceRad = Units.degreesToRadians(10);

    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 3);

    // 0 = parallel with ground
    static final double minPositionRad = Units.degreesToRadians(0);
    static final double maxPositionRad = Units.degreesToRadians(90);
    static final double initialPositionRad = minPositionRad;

    static final double gearRatio = 120;
    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY, SIM -> new LoggedTunablePIDF("ExampleServoSubsystem/Gains");
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
