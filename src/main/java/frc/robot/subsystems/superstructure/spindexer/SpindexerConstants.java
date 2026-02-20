package frc.robot.subsystems.superstructure.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.robot.BuildConstants;

public class SpindexerConstants {
    static final double gearRatio = 5;

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    -1,
                    true,
                    true,
                    40,
                    gearRatio,
                    null,
                    null
            );
            case SIM -> new MotorIOSim(
                    gearRatio,
                    0.01,
                    DCMotor.getNEO(1),
                    null,
                    null
            );
            case REPLAY -> new MotorIO();
        };
    }
}
