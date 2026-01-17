package frc.robot.subsystems.superintake.intakerollers;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.PIDF;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.motor.RequestTolerances;
import frc.robot.BuildConstants;

public class IntakeRollersConstants {
    static final double gearRatio = 5;

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    -1,
                    true,
                    true,
                    40,
                    gearRatio,
                    PIDF.zero(),
                    PIDF.zero()
            );
            case SIM -> new MotorIOSim(
                    gearRatio,
                    0.01,
                    DCMotor.getNEO(1),
                    PIDF.zero(),
                    PIDF.zero()
            );
            case REPLAY -> new MotorIO();
        };
    }
}
