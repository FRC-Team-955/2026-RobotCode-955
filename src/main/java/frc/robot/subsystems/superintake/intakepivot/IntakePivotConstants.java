package frc.robot.subsystems.superintake.intakepivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.PIDF;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOArmSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.robot.BuildConstants;

public class IntakePivotConstants {
    static final double positionToleranceRad = Units.degreesToRadians(10);

    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 3);

    static final double gearRatio = 120;
    static final PIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> PIDF.zero();
        case SIM -> PIDF.ofPDSG(20.0, 0.0, 0.0, 2.65);
    };

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    -1,
                    true,
                    true,
                    40,
                    gearRatio,
                    gains,
                    PIDF.zero()
            );
            case SIM -> new MotorIOArmSim(
                    DCMotor.getNEO(1),
                    gearRatio,
                    1.4,
                    0.3,
                    Units.degreesToRadians(-90),
                    Units.degreesToRadians(90),
                    true,
                    Units.degreesToRadians(80),
                    0.001,
                    gains
            );
            case REPLAY -> new MotorIO();
        };
    }
}
