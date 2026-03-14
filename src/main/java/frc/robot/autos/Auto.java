package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AllianceFlipUtil;
import frc.lib.commands.CommandsExt;
import frc.robot.RobotState;

public abstract class Auto {
    private static final RobotState robotState = RobotState.get();

    public final Pose2d startingPose;
    public final Command command;

    protected Auto(Pose2d startingPose, Command command) {
        this.startingPose = startingPose;
        this.command = CommandsExt.eagerSequence(
                robotState.setPose(() -> AllianceFlipUtil.apply(startingPose)),
                command
        );
    }
}
