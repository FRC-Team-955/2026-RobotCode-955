package frc.robot;

import frc.lib.subsystem.Periodic;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class RobotMechanism implements Periodic {
    @Getter
    private static final RobotMechanism instance = new RobotMechanism();

    private RobotMechanism() {
    }

    @Override
    public void periodicAfterCommands() {
        // All transforms are relative to center of robot at the bottom of the frame rail
        Logger.recordOutput("RobotMechanism/Pose", RobotState.getInstance().getMechanismPose());
        Logger.recordOutput(
                "RobotMechanism/Components",
                Superintake.getInstance().intakeRollers.getMechanismTransform(),
                Superintake.getInstance().intakePivot.getMechanismTransform(),
                Superstructure.getInstance().spindexer.getMechanismTransform(),
                Superstructure.getInstance().feeder.getMechanismTransform(),
                Superstructure.getInstance().flywheel.getMechanismTransform(),
                Superstructure.getInstance().hood.getMechanismTransform()
        );
    }
}