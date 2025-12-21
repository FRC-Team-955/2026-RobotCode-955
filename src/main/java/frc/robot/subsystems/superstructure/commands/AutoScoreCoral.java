package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.superstructure.ReefAlign;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureCommand;
import lombok.RequiredArgsConstructor;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.scoreCoralL1SettleSeconds;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.scoreCoralSettleSeconds;

@RequiredArgsConstructor
public class AutoScoreCoral extends SuperstructureCommand {
    private final Supplier<ReefAlign.ReefZoneSide> reefSideSupplier;
    private final Supplier<ReefAlign.LocalReefSide> sideSupplier;
    private final Supplier<OperatorDashboard.CoralScoringLevel> coralScoringLevelSupplier;
    private final BooleanSupplier forceCondition;

    @Override
    public Command create() {
        DoubleSupplier elevatorPercentageSupplier = () -> elevator.getPositionMeters() / coralScoringLevelSupplier.get().coralScoringElevatorGoal.value.getAsDouble();
        Supplier<Pose2d> alignPoseSupplier = () -> ReefAlign.getAlignPose(robotState.getPose(), elevatorPercentageSupplier.getAsDouble(), reefSideSupplier.get(), sideSupplier.get());

        Command initial = Commands.race(
                // Drive to initial position
                drive.moveTo(alignPoseSupplier, false),
                Commands.parallel(
                        superstructure.setGoal(
                                Superstructure.Goal.AUTO_SCORE_CORAL_WAIT_UNTIL_CAN_RAISE,
                                () -> Elevator.Goal.STOW,
                                () -> EndEffector.Goal.IDLE,
                                Funnel.Goal.IDLE
                        )
                ),
                Commands.waitUntil(() -> ReefAlign.canRaiseElevator(robotState.getPose(), reefSideSupplier.get(), sideSupplier.get()))
        );

        Command waitFinalAndElevator = CommandsExt.eagerSequence(
                Commands.parallel(
                        superstructure.setGoal(
                                Superstructure.Goal.AUTO_SCORE_CORAL_WAIT_FOR_ALIGN,
                                () -> coralScoringLevelSupplier.get().coralScoringElevatorGoal,
                                () -> EndEffector.Goal.IDLE,
                                Funnel.Goal.IDLE
                        ),
                        Commands.waitUntil(() -> ReefAlign.atFinalAlign(robotState.getPose(), robotState.getMeasuredChassisSpeeds(), reefSideSupplier.get(), sideSupplier.get()))
                ),
                Commands.parallel(
                        superstructure.setGoal(
                                Superstructure.Goal.AUTO_SCORE_CORAL_WAIT_FOR_ELEVATOR,
                                () -> coralScoringLevelSupplier.get().coralScoringElevatorGoal,
                                () -> EndEffector.Goal.IDLE,
                                Funnel.Goal.IDLE
                        ),
                        elevator.waitUntilAtGoal()
                )
        );
        // Don't allow forcing for a bit, then check if force is true
        Command waitForForceAndAlign = CommandsExt.eagerSequence(
                Commands.waitSeconds(2).deadlineFor(drive.moveTo(alignPoseSupplier, false)),
                Commands.waitUntil(forceCondition).deadlineFor(
                        rumble(),
                        drive.moveTo(alignPoseSupplier, true),
                        // We only want to offset the elevator position if we aren't aligned and are taking a while to align
                        elevator.setDistanceFromScoringPositionContinuous(
                                () -> robotState.getPose().getTranslation()
                                        .getDistance(ReefAlign.getFinalAlignPose(reefSideSupplier.get(), sideSupplier.get()).getTranslation())
                        )
                )
        );

        Command score = CommandsExt.eagerSequence(
                superstructure.setGoal(
                        Superstructure.Goal.AUTO_SCORE_CORAL_WAIT_BEFORE_SCORING,
                        () -> coralScoringLevelSupplier.get().coralScoringElevatorGoal,
                        () -> EndEffector.Goal.IDLE,
                        Funnel.Goal.IDLE
                ),
                Commands.waitSeconds(0.3),
                superstructure.setGoal(
                        Superstructure.Goal.AUTO_SCORE_CORAL_SCORING,
                        () -> coralScoringLevelSupplier.get().coralScoringElevatorGoal,
                        () -> coralScoringLevelSupplier.get() == OperatorDashboard.CoralScoringLevel.L1
                                ? EndEffector.Goal.SCORE_CORAL_L1
                                : EndEffector.Goal.SCORE_CORAL,
                        Funnel.Goal.IDLE
                ),
                waitUntilEndEffectorNotTriggered()
        );
        // Wait for coral to settle and send the elevator back down
        Command finalize = Commands.either(
                Commands.waitSeconds(scoreCoralL1SettleSeconds),
                Commands.waitSeconds(scoreCoralSettleSeconds),
                () -> coralScoringLevelSupplier.get() == OperatorDashboard.CoralScoringLevel.L1
        );

        return CommandsExt.eagerSequence(
                aprilTagVision.setTagIdFilter(ReefAlign.reefTagIds),
                initial,
                Commands.race(
                        waitFinalAndElevator,
                        waitForForceAndAlign
                ),
                // At this point, the move to goal is already running so no need to set it again
                CommandsExt.eagerSequence(
                        score,
                        finalize
                )
        );
    }
}
