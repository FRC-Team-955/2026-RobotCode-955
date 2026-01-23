package frc.gametest;

import java.util.HashMap;
import java.util.Map;

class GameTests {
    static final Map<String, GameTest> gameTests = new HashMap<>();

    static {
//        for (int branch = 0; branch < 12; branch++) {
//            // make a copy for use in the lambda
//            int branchFinal = branch;
//
//            var startingPose = new Pose2d(branch < 4 || branch >= 10 ? 2 : 6, branch < 6 ? 2 : 6, new Rotation2d());
//            var reefSide = branch >= 10 ? ReefAlign.ReefZoneSide.LeftFront : (
//                    branch >= 8 ? ReefAlign.ReefZoneSide.LeftBack : (
//                            branch >= 6 ? ReefAlign.ReefZoneSide.MiddleBack : (
//                                    branch >= 4 ? ReefAlign.ReefZoneSide.RightBack : (
//                                            branch >= 2 ? ReefAlign.ReefZoneSide.RightFront : ReefAlign.ReefZoneSide.MiddleFront
//                                    )
//                            )
//                    )
//            );
//            var localSide = branch % 2 == 0 ? ReefAlign.LocalReefSide.Left : ReefAlign.LocalReefSide.Right;
//
//            for (int level = 1; level < 4; level++) {
//                if (level != 3) {
//                    // TODO: everything except L4 is broken
//                    continue;
//                }
//
//                // make a copy for use in the lambda
//                int levelFinal = level;
//
//                var scoringLevel = level == 0 ? OperatorDashboard.CoralScoringLevel.L1 : (
//                        level == 1 ? OperatorDashboard.CoralScoringLevel.L2 : (
//                                level == 2 ? OperatorDashboard.CoralScoringLevel.L3 : OperatorDashboard.CoralScoringLevel.L4
//                        )
//                );
//
//                gameTests.put("Score coral on branch " + branch + " at level L" + (level + 1), new GameTest(
//                        startingPose,
//                        Superstructure.get().autoScoreCoral(
//                                () -> reefSide,
//                                () -> localSide,
//                                () -> scoringLevel,
//                                () -> false
//                        ),
//                        1500,
//                        () -> ReefscapeReefSimulation.getInstance().orElseThrow().getBranches(DriverStation.Alliance.Blue)
//                                [branchFinal]
//                                [levelFinal] == 1
//                ));
//            }
//        }
    }
}
