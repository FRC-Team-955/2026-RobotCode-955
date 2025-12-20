package frc.gametest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.superstructure.ReefAlign;
import frc.robot.subsystems.superstructure.Superstructure;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeReefSimulation;

import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;

class GameTests {
    static final Map<String, GameTest> gameTests = new HashMap<>();

    static {
        for (int i = 0; i < 12; i++) {
            // make a copy for use in the lambda
            int final_i = i;

            gameTests.put("Score coral L4 " + i, new GameTest(
                    new Pose2d(i < 4 || i >= 10 ? 2 : 6, i < 6 ? 2 : 6, new Rotation2d()),
                    Superstructure.get().autoScoreCoral(
                            () -> final_i >= 10 ? ReefAlign.ReefZoneSide.LeftFront : (
                                    final_i >= 8 ? ReefAlign.ReefZoneSide.LeftBack : (
                                            final_i >= 6 ? ReefAlign.ReefZoneSide.MiddleBack : (
                                                    final_i >= 4 ? ReefAlign.ReefZoneSide.RightBack : (
                                                            final_i >= 2 ? ReefAlign.ReefZoneSide.RightFront : ReefAlign.ReefZoneSide.MiddleFront
                                                    )
                                            )
                                    )
                            ),
                            () -> final_i % 2 == 0 ? ReefAlign.LocalReefSide.Left : ReefAlign.LocalReefSide.Right,
                            () -> OperatorDashboard.CoralScoringLevel.L4,
                            () -> false
                    ),
                    1500,
                    () -> assertEquals(1,
                            ReefscapeReefSimulation.getInstance().get().getBranches(DriverStation.Alliance.Blue)
                                    [final_i] // branch C
                                    [3] // L4
                    )
            ));
        }
    }
}
