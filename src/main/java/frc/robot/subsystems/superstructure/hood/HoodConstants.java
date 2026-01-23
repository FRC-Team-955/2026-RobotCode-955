package frc.robot.subsystems.superstructure.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.PIDF;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.motor.RequestTolerances;
import frc.robot.BuildConstants;

public class HoodConstants {
    static final RequestTolerances tolerances = RequestTolerances.defaults();

    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 3);

    static final double gearRatio = 120;
    static final PIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> PIDF.zero();
        case SIM -> PIDF.zero();
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
            case SIM -> new MotorIOSim(
                    gearRatio,
                    0.01,
                    DCMotor.getNEO(1),
                    gains,
                    PIDF.zero()
            );
            case REPLAY -> new MotorIO();
        };
    }
}
