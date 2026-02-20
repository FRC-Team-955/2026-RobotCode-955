package frc.robot.subsystems.superstructure.feeder;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.robot.BuildConstants;

public class FeederConstants {
    static final double gearRatio = 5;

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    -1,
                    true,
                    SparkBaseConfig.IdleMode.kCoast,
                    40,
                    gearRatio,
                    null,
                    null,
                    0.0
            );
            case SIM -> new MotorIOSim(
                    gearRatio,
                    0.01,
                    DCMotor.getNEO(1),
                    null,
                    null,
                    0.0
            );
            case REPLAY -> new MotorIO();
        };
    }
}
