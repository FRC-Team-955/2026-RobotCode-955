package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.subsystem.Periodic;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class RobotMechanism implements Periodic {
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
                        new Translation3d(0.0, 0.0, driveConfig.bottomOfFrameRailsToCenterOfWheelsMeters() + driveConfig.wheelRadiusMeters()),
                        new Rotation3d()
                ));

        Logger.recordOutput("RobotMechanism/Pose", robotPose);
    }
}