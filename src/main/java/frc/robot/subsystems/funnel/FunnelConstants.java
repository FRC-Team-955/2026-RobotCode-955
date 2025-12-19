package frc.robot.subsystems.funnel;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.PIDF;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.motor.RequestTolerances;
import frc.robot.BuildConstants;

public class FunnelConstants {
    static final RequestTolerances tolerances = RequestTolerances.defaults();

    static final double gearRatio = 5;
    static final PIDF velocityGains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> PIDF.ofPSV(0.01, 0.21416, 0.10077);
        case SIM -> PIDF.ofSV(0.00995, 0.17859);
    };

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    5,
                    true,
                    true,
                    40,
                    gearRatio,
                    PIDF.ofP(0.2),
                    velocityGains
            );
            case SIM -> new MotorIOSim(
                    gearRatio,
                    0.01,
                    DCMotor.getNEO(1),
                    PIDF.ofP(1.5),
                    velocityGains
            );
            case REPLAY -> new MotorIO();
        };
    }
}
