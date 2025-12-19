package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard.CoralScoringLevel;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.superstructure.ReefAlign.LocalReefSide;
import frc.robot.subsystems.superstructure.ReefAlign.ReefZoneSide;
import frc.robot.subsystems.superstructure.StationAlign.Station;

import java.util.List;

public class ProcessorSideFriendlyAuto {
    public static Command get() {
        final var firstScoreTraj = AutoHelper.trajectory("Processor Side Friendly", 0);
        final var secondStationTraj = AutoHelper.trajectory("Processor Side Friendly", 1);
        final var secondScoreTraj = AutoHelper.trajectory("Processor Side Friendly", 2);
        final var thirdStationTraj = AutoHelper.trajectory("Processor Side Friendly", 3);
        final var thirdScoreTraj = AutoHelper.trajectory("Processor Side Friendly", 4);
        final var fourthStationTraj = AutoHelper.trajectory("Processor Side Friendly", 5);
        final var fourthScoreTraj = AutoHelper.trajectory("Processor Side Friendly", 6);

        return AutoBuilder.createScoring(List.of(
                new IntakeScorePair(null, null, firstScoreTraj, ReefZoneSide.MiddleFront, LocalReefSide.Right, CoralScoringLevel.L4),
                new IntakeScorePair(secondStationTraj, Station.ProcessorSideFriendly, secondScoreTraj, ReefZoneSide.MiddleFront, LocalReefSide.Left, CoralScoringLevel.L4),
                new IntakeScorePair(thirdStationTraj, Station.ProcessorSideFriendly, thirdScoreTraj, ReefZoneSide.MiddleFront, LocalReefSide.Right, CoralScoringLevel.L2),
                new IntakeScorePair(fourthStationTraj, Station.ProcessorSideFriendly, fourthScoreTraj, ReefZoneSide.MiddleFront, LocalReefSide.Left, CoralScoringLevel.L2)
        ));
    }
}
