package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureCommand;
import lombok.RequiredArgsConstructor;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.scoreCoralL1SettleSeconds;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.scoreCoralSettleSeconds;

@RequiredArgsConstructor
public class ScoreCoralManual extends SuperstructureCommand {
    private final BooleanSupplier forwardCondition;
    private final Supplier<OperatorDashboard.CoralScoringLevel> coralScoringLevelSupplier;

    @Override
    public Command create() {
        Command waitConfirm = Commands.parallel(
                superstructure.setGoal(
                        Superstructure.Goal.MANUAL_SCORE_CORAL_WAIT_FOR_CONFIRM,
                        () -> coralScoringLevelSupplier.get().coralScoringElevatorGoal,
                        () -> EndEffector.Goal.IDLE,
                        Funnel.Goal.IDLE
                ),
                rumble(),
                Commands.waitUntil(forwardCondition)
        );

        Command driveWhileScoringL1 = CommandsExt.onlyIf(
                () -> coralScoringLevelSupplier.get() == OperatorDashboard.CoralScoringLevel.L1,
                drive.runRobotRelative(() -> new ChassisSpeeds(0, -1.0, 0))
        );

        Command score = Commands.parallel(
                superstructure.setGoal(
                        Superstructure.Goal.MANUAL_SCORE_CORAL_SCORING,
                        () -> coralScoringLevelSupplier.get().coralScoringElevatorGoal,
                        () -> coralScoringLevelSupplier.get() == OperatorDashboard.CoralScoringLevel.L1
                                ? EndEffector.Goal.SCORE_CORAL_L1
                                : EndEffector.Goal.SCORE_CORAL,
                        Funnel.Goal.IDLE
                ),
                waitUntilEndEffectorNotTriggered()
        );

        // Wait for coral to settle
        Command finalize = Commands.either(
                Commands.waitSeconds(scoreCoralL1SettleSeconds),
                Commands.waitSeconds(scoreCoralSettleSeconds),
                () -> coralScoringLevelSupplier.get() == OperatorDashboard.CoralScoringLevel.L1
        );

        return CommandsExt.eagerSequence(
                superstructure.setGoal(
                        Superstructure.Goal.MANUAL_SCORE_CORAL_WAIT_FOR_ELEVATOR,
                        () -> coralScoringLevelSupplier.get().coralScoringElevatorGoal,
                        () -> EndEffector.Goal.IDLE,
                        Funnel.Goal.IDLE
                ),
                elevator.waitUntilAtGoal(),
                waitConfirm,
                CommandsExt.eagerSequence(
                        score,
                        finalize
                ).deadlineFor(driveWhileScoringL1)
        );
    }
}
