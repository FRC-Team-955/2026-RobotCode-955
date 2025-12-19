package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard.CoralScoringLevel;
import frc.robot.autos.AutoBuilder.IntakeScorePair;
import frc.robot.subsystems.superstructure.ReefAlign.LocalReefSide;
import frc.robot.subsystems.superstructure.ReefAlign.ReefZoneSide;
import frc.robot.subsystems.superstructure.StationAlign.Station;

import java.util.List;

public class BargeSideAuto {
    public static Command get(Type type) {
        return AutoBuilder.createScoring(List.of(
                new IntakeScorePair(
                        null,
                        null,
                        AutoHelper.trajectory("Barge Side", 0),
                        ReefZoneSide.LeftBack,
                        LocalReefSide.Right,
                        CoralScoringLevel.L4
                ),
                new IntakeScorePair(
                        AutoHelper.trajectory("Barge Side", 1),
                        Station.BargeSide,
                        AutoHelper.trajectory("Barge Side", 2),
                        ReefZoneSide.LeftFront,
                        LocalReefSide.Left,
                        CoralScoringLevel.L4
                ),

                switch (type) {
                    case Normal, AvoidMiddleFront -> new IntakeScorePair(
                            AutoHelper.trajectory("Barge Side", 3),
                            Station.BargeSide,
                            AutoHelper.trajectory("Barge Side", 4),
                            ReefZoneSide.LeftFront,
                            LocalReefSide.Right,
                            CoralScoringLevel.L4
                    );
                    case AvoidMiddleFrontAndAdjacent -> new IntakeScorePair(
                            AutoHelper.trajectory("Barge Side", 3),
                            Station.BargeSide,
                            AutoHelper.trajectory("Barge Side - Alternate Left"),
                            ReefZoneSide.LeftBack,
                            LocalReefSide.Left,
                            CoralScoringLevel.L4
                    );
                },
                switch (type) {
                    case Normal -> new IntakeScorePair(
                            AutoHelper.trajectory("Barge Side", 5),
                            Station.BargeSide,
                            AutoHelper.trajectory("Barge Side", 6),
                            ReefZoneSide.MiddleFront,
                            LocalReefSide.Left,
                            CoralScoringLevel.L2
                    );
                    case AvoidMiddleFront -> new IntakeScorePair(
                            AutoHelper.trajectory("Barge Side", 5),
                            Station.BargeSide,
                            AutoHelper.trajectory("Barge Side - Alternate Right"),
                            ReefZoneSide.LeftBack,
                            LocalReefSide.Right,
                            CoralScoringLevel.L2
                    );
                    case AvoidMiddleFrontAndAdjacent -> new IntakeScorePair(
                            AutoHelper.trajectory("Barge Side", 1),
                            Station.BargeSide,
                            AutoHelper.trajectory("Barge Side - Alternate Right"),
                            ReefZoneSide.LeftBack,
                            LocalReefSide.Right,
                            CoralScoringLevel.L2
                    );
                }
        ));
    }

    public enum Type {
        Normal,
        AvoidMiddleFront,
        AvoidMiddleFrontAndAdjacent,
    }
}
