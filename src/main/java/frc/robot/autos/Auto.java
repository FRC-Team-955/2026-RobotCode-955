package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AllianceFlipUtil;
import frc.lib.commands.CommandsExt;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;

public abstract class Auto {
    protected static final Drive drive = Drive.getInstance();
    protected static final Superintake superintake = Superintake.getInstance();
    protected static final Superstructure superstructure = Superstructure.getInstance();
    protected static final RobotState robotState = RobotState.getInstance();
    protected static final OperatorDashboard operatorDashboard = OperatorDashboard.getInstance();

    public final Pose2d startingPose;
    public final Command command;

    protected Auto(Pose2d startingPose, Command command) {
        this.startingPose = startingPose;
        this.command = CommandsExt.eagerSequence(
                CommandsExt.suppliedWaitSeconds(operatorDashboard.autoDelay::get),
                robotState.setPose(() -> AllianceFlipUtil.apply(startingPose)),
                command
        );
    }
}
