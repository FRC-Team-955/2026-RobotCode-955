package frc.robot.subsystems.superstructure.turret;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.MotorIOSparkMax;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

public class TurretConstants {
    static final double positionToleranceRad = Units.degreesToRadians(10);

    static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 3);

    /** Only used before homing, and in sim, where nobody can push the turret into a stop */
    static final double initialPositionRad = 0;

    /**
     * Travel to assume in sim, where there are no hard stops to home against. On the real robot
     * the travel is measured during homing instead of being declared here.
     */
    static final double simTravelRad = Units.degreesToRadians(360);

    /**
     * Homing is rejected below this much measured travel. Catches the stray double-press that
     * would otherwise record both stops at the same spot and pin the turret to a zero-width range.
     */
    static final double minimumPlausibleTravelRad = Units.degreesToRadians(10);
    
    static final double gearRatio = 3.0 * 3.0 * (68.0 / 12.0);
    static final LoggedTunablePIDF gains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Turret/Gains")
                .withP(0.5)
                .withD(0.0);
        case SIM -> new LoggedTunablePIDF("Superstructure/Turret/Gains")
                .withP(30.0)
                .withD(0.0);
    };

    static MotorIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(
                    11,
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
