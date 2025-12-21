package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.superstructure.ReefAlign;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureCommand;
import lombok.RequiredArgsConstructor;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@RequiredArgsConstructor
public class AutoDescoreAlgae extends SuperstructureCommand {
    private final Supplier<ReefAlign.ReefZoneSide> reefZoneSideSupplier;
    private final BooleanSupplier forceCondition;

    @Override
    public Command create() {
        Command driveTo = Commands.race(
                // Drive to position
                drive.moveTo(() -> ReefAlign.getDescoreAlignPose(reefZoneSideSupplier.get()), false),
                CommandsExt.eagerSequence(
                        Commands.parallel(
                                superstructure.setGoal(
                                        Superstructure.Goal.AUTO_DESCORE_ALGAE_WAIT_UNTIL_CAN_RAISE,
                                        () -> Elevator.Goal.STOW,
                                        () -> EndEffector.Goal.IDLE,
                                        Funnel.Goal.IDLE
                                ),
                                Commands.waitUntil(() -> ReefAlign.descoreCanRaiseElevator(reefZoneSideSupplier.get()))
                        ),
                        Commands.parallel(
                                superstructure.setGoal(
                                        Superstructure.Goal.AUTO_DESCORE_ALGAE_WAIT_FOR_ALIGN,
                                        () -> reefZoneSideSupplier.get().algaeDescoringElevatorGoal,
                                        () -> EndEffector.Goal.DESCORE_ALGAE,
                                        Funnel.Goal.IDLE
                                ),
                                Commands.waitUntil(() -> ReefAlign.descoreIsAligned(reefZoneSideSupplier.get()))
                        )
                )
        );

        Command waitAlgae = Commands.parallel(
                Commands.race(
                        drive.runRobotRelative(() -> new ChassisSpeeds(-0.4, 0, 0)),
                        CommandsExt.eagerSequence(
                                Commands.waitSeconds(0.5),
                                endEffector.waitUntilDescoreAlgaeAmperageTriggered()
                        )
                ),
                superstructure.setGoal(
                        Superstructure.Goal.AUTO_DESCORE_ALGAE_WAIT_FOR_AMPERAGE,
                        () -> reefZoneSideSupplier.get().algaeDescoringElevatorGoal,
                        () -> EndEffector.Goal.DESCORE_ALGAE,
                        Funnel.Goal.IDLE
                )
        );

        Timer driveBackTimer = new Timer();
        Command driveBack = drive.runRobotRelative(() -> new ChassisSpeeds(driveBackTimer.get() * 4.0, 0, 0))
                .withTimeout(0.5)
                .deadlineFor(
                        Commands.runOnce(driveBackTimer::restart),
                        superstructure.setGoal(
                                Superstructure.Goal.AUTO_DESCORE_ALGAE_MOVE_BACK,
                                () -> reefZoneSideSupplier.get().algaeDescoringElevatorGoal,
                                () -> EndEffector.Goal.DESCORE_ALGAE,
                                Funnel.Goal.IDLE
                        )
                );

        Command waitForForce = CommandsExt.eagerSequence(
                Commands.waitSeconds(2),
                Commands.waitUntil(forceCondition)
        );

        return CommandsExt.eagerSequence(
                aprilTagVision.setTagIdFilter(ReefAlign.reefTagIds),
                Commands.race(
                        CommandsExt.eagerSequence(
                                driveTo,
                                waitAlgae
                        ),
                        waitForForce
                ),
                driveBack
        );
    }
}
