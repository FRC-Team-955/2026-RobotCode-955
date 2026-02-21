package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.BuildConstants;

public class SuperstructureConstants {
    static final Transform3d robotToCANrange = new Transform3d(
            // Z is from carpet
            new Translation3d(Units.inchesToMeters(-4.446890), Units.inchesToMeters(-14.139000), Units.inchesToMeters(7)),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0))
    );

    static SuperstructureIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new SuperstructureIOCANrange();
            case SIM -> new SuperstructureIOSim();
            case REPLAY -> new SuperstructureIO();
        };
    }
}