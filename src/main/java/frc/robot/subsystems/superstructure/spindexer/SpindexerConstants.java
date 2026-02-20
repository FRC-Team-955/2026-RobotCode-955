package frc.robot.subsystems.superstructure.spindexer;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOTalonFX;
import frc.robot.BuildConstants;

public class SpindexerConstants {
    static final double gearRatio = 5;

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOTalonFX(
                    -1,
                    true,
                    NeutralModeValue.Coast,
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
