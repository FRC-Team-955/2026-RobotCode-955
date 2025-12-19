package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.CommandsExt;
import frc.robot.OperatorDashboard.CoralScoringLevel;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.superstructure.ReefAlign.LocalReefSide;
import frc.robot.subsystems.superstructure.ReefAlign.ReefZoneSide;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.List;

public class CenterAuto {
    public static Command get(Type type) {
        final var firstScoreTraj = AutoHelper.trajectory("Center");

        Command auto = AutoBuilder.createScoring(List.of(
                new IntakeScorePair(null, null, firstScoreTraj, ReefZoneSide.MiddleBack, LocalReefSide.Left, CoralScoringLevel.L4)
        ));

        return switch (type) {
            case Descore -> CommandsExt.eagerSequence(
                    auto,
                    Superstructure.get().autoDescoreAlgae(() -> ReefZoneSide.MiddleBack, () -> true).asProxy()
            );
            case Normal -> auto;
        };
    }

    public enum Type {
        Normal,
        Descore,
    }
}
