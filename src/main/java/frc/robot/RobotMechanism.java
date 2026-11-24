package frc.robot;

import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

public class RobotMechanism implements Periodic {
    private static final RobotState robotState = RobotState.get();
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
        // All transforms are relative to center of robot at the bottom of the frame rail
        Logger.recordOutput("RobotMechanism/Pose", robotState.getMechanismPose());
        Logger.recordOutput(
                "RobotMechanism/Components",
                superintake.intakeRollers.getMechanismTransform(),
                superstructure.spindexer.getMechanismTransform(),
                superstructure.feeder.getMechanismTransform(),
                superstructure.flywheel.getMechanismTransform(),
                superintake.intakePivot.getMechanismTransform(),
                superstructure.hood.getMechanismTransform()
        );
    }
}