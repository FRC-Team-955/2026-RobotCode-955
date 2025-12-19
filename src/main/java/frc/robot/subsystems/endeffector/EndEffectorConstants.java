package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.PIDF;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.motor.RequestTolerances;
import frc.robot.BuildConstants;

public class EndEffectorConstants {
    static final RequestTolerances tolerances = RequestTolerances.position(Units.degreesToRadians(15));
    static final double rollersRadiusMeters = Units.inchesToMeters(2.25 / 2.0);

    static final double descoreAlgaeTriggerAmps = 27;

    static final double extendStartMeters = Units.inchesToMeters(5);
    static final double extendDistanceMeters = Units.inchesToMeters(2.25);
    static final double angleWhenExtendedRad = Units.degreesToRadians(40);
    static final double angleWhenRetractedRad = Units.degreesToRadians(90);

    static double rollersRadiansForMeters(double meters) {
        return meters / rollersRadiusMeters;
    }

    static final double gearRatio = 9;
    static final PIDF positionGains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> PIDF.ofP(0.5);
        case SIM -> PIDF.ofP(3);
    };
    static final PIDF velocityGains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> PIDF.ofPSV(0.01, 0.42461, 0.18272);
        case SIM -> PIDF.ofSV(0.00995, 0.17859);
    };

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    6,
                    false,
                    true,
                    40,
                    gearRatio,
                    positionGains,
                    velocityGains
            );
            case SIM -> new MotorIOSim(
                    gearRatio,
                    0.01,
                    DCMotor.getNEO(1),
                    positionGains,
                    velocityGains
            );
            case REPLAY -> new MotorIO();
        };
    }
}