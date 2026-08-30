package frc.robot;

import frc.lib.subsystem.Periodic;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class RobotMechanism implements Periodic {
    private static final RobotState robotState = RobotState.getInstance();
    private static final Superintake superintake = Superintake.getInstance();
    private static final Superstructure superstructure = Superstructure.getInstance();

    @Getter
    private static final RobotMechanism instance = new RobotMechanism();

    private RobotMechanism() {
    }

    @Override
    public void periodicAfterCommands() {
        // All transforms are relative to center of robot at the bottom of the frame rail
        Logger.recordOutput("RobotMechanism/Pose", robotState.getMechanismPose());
        Logger.recordOutput(
                "RobotMechanism/Components",
                superintake.intakeRollers.getMechanismTransform(),
                superintake.intakePivot.getMechanismTransform(),
                superstructure.spindexer.getMechanismTransform(),
                superstructure.feeder.getMechanismTransform(),
                superstructure.flywheel.getMechanismTransform(),
                superstructure.hood.getMechanismTransform()
        );
    }
}