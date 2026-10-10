package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

public class RobotMechanism implements Periodic {
    private static final RobotState robotState = RobotState.get();
    private static final Drive drive = Drive.get();
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static RobotMechanism instance;

    public static synchronized RobotMechanism get() {
        if (instance == null) {
            instance = new RobotMechanism();
        }

        return instance;
    }

    private RobotMechanism() {
        if (instance != null) {
            Util.error("Duplicate RobotMechanism created");
        }
    }

    @Override
    public void periodicAfterCommands() {
        Pose3d pose = robotState.getMechanismPose();
        pose = new Pose3d(pose.getTranslation(), pose.getRotation().rotateBy(drive.getGyroPitchRollRotation()));
        Logger.recordOutput("RobotMechanism/Pose", pose);

        // All transforms are relative to center of robot at the bottom of the frame rail
        Logger.recordOutput(
                "RobotMechanism/Components",
                superintake.intakeRollers.getMechanismTransform(),
                superstructure.spindexer.getMechanismTransform(),
                superstructure.feeder.getMechanismTransform(),
                superstructure.flywheel.getMechanismTransform(),
                superintake.intakePivot.getMechanismTransform(),
                superstructure.hood.getMechanismTransform(),
                superstructure.turret.getMechanismTransform()
        );
    }
}