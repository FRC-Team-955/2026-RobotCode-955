package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.subsystem.Periodic;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class RobotMechanism implements Periodic {
    // All transforms are relative to center of robot at the bottom of the frame rail
    private static final Transform3d intakeRollerInitial = new Transform3d(
            new Translation3d(Units.inchesToMeters(14.0), 0.0, Units.inchesToMeters(4.0)),
            new Rotation3d()
    );
    private static final Transform3d indexerInitial = new Transform3d(
            new Translation3d(0.0, 0.0, Units.inchesToMeters(6.0)),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0))
    );
    private static final Transform3d flywheelsInitial = new Transform3d(
            new Translation3d(Units.inchesToMeters(-5.0), Units.inchesToMeters(-9.0), Units.inchesToMeters(10.0)),
            new Rotation3d(0.0, 0.0, 0.0)
    );
    private static final Transform3d intakePivotInitial = new Transform3d(
            new Translation3d(Units.inchesToMeters(12.0), 0.0, Units.inchesToMeters(4.0)),
            new Rotation3d(0.0, 0.0, 0.0)
    );
    private static final Transform3d hoodInitial = new Transform3d(
            new Translation3d(Units.inchesToMeters(-4.0), Units.inchesToMeters(-9.0), Units.inchesToMeters(12.0)),
            new Rotation3d(0.0, Units.degreesToRadians(-90.0), 0.0)
    );

    private static final RobotState robotState = RobotState.get();

    private static RobotMechanism instance;

    public static RobotMechanism get() {
        if (instance == null)
            synchronized (RobotMechanism.class) {
                instance = new RobotMechanism();
            }

        return instance;
    }

    private RobotMechanism() {
    }

    @Override
    public void periodicAfterCommands() {
        Pose3d robotPose = new Pose3d(robotState.getPose())
                .transformBy(new Transform3d(
                        new Translation3d(0.0, 0.0, driveConfig.bottomOfFrameToCenterOfWheelsMeters() + driveConfig.wheelRadiusMeters()),
                        new Rotation3d()
                ));

        Logger.recordOutput("RobotMechanism/Pose", robotPose);
        Logger.recordOutput(
                "RobotMechanism/Components",
                intakeRollerInitial,
                indexerInitial,
                flywheelsInitial,
                intakePivotInitial,
                hoodInitial
        );
    }
}