package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.superstructure.StationAlign;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureCommand;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoFunnelIntake extends SuperstructureCommand {
    private final StationAlign.Station station;

    @Override
    public Command create() {
        Command intake = Commands.race(
                waitUntilEndEffectorTriggered(),
                waitUntilFunnelTriggered(),
                gamePieceVision.waitForGamePiece()
        ).deadlineFor(
                CommandsExt.eagerSequence(
                        Commands.parallel(
                                superstructure.setGoal(
                                        Superstructure.Goal.AUTO_FUNNEL_INTAKE_WAITING_ALIGN,
                                        () -> Elevator.Goal.STOW,
                                        () -> EndEffector.Goal.FUNNEL_INTAKE,
                                        Funnel.Goal.INTAKE_ALTERNATE
                                ),
                                drive.moveTo(station::getAlignPose, false)
                                        .until(() -> StationAlign.atAlignPose(robotState.getPose(), station))
                        ),
                        Commands.parallel(
                                superstructure.setGoal(
                                        Superstructure.Goal.AUTO_FUNNEL_INTAKE_WAITING_SHAKE,
                                        () -> Elevator.Goal.STOW,
                                        () -> EndEffector.Goal.FUNNEL_INTAKE,
                                        Funnel.Goal.INTAKE_ALTERNATE
                                ),
                                shake()
                        )
                )
        );
        return CommandsExt.eagerSequence(
                intake,
                waitUntilEndEffectorTriggered().deadlineFor(
                        superstructure.setGoal(
                                Superstructure.Goal.HANDOFF,
                                () -> Elevator.Goal.STOW,
                                () -> EndEffector.Goal.FUNNEL_INTAKE,
                                Funnel.Goal.INTAKE_ALTERNATE
                        )
                ),
                superstructure.home()
        );
    }
}
