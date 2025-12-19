package frc.robot.autos;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.ReefAlign;
import frc.robot.subsystems.superstructure.StationAlign;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.List;

public class AutoBuilder {
    public static Command createScoring(List<IntakeScorePair> trajectories) {
        final Superstructure superstructure = Superstructure.get();
        final Drive drive = Drive.get();

        if (trajectories.isEmpty()) {
            return Commands.none();
        }

        IntakeScorePair first = trajectories.get(0);

        Command startCmd = CommandsExt.eagerSequence(
                AutoHelper.resetOdometry(first.scoreTraj),
                drive.followTrajectory(first.scoreTraj)
        );

        IntakeScorePair last = first;
        // If there's only one trajectory, skip to the finish score
        if (trajectories.size() > 1) {
            for (IntakeScorePair next : trajectories) {
                // Skip first trajectory
                if (next == first || next.station == null || next.stationTraj == null) continue;

//                last.scoreTraj.atTime("score").onTrue(CommandsExt.eagerSequence(
//                        last.scoreCommand(superstructure),
//                        // scheduling the trajectory wastes a cycle; instead, reset the superstructure and run the trajectory at the same time
//                        Commands.parallel(
//                                // TODO dont need this or wait?
//                                superstructure.cancel(),
//                                drive.followTrajectory(next.stationTraj)
//                        )
//                ));

//                next.stationTraj.atTime("intake").onTrue(CommandsExt.eagerSequence(
//                        superstructure.autoFunnelIntake(next.station), // TODO precondition !endEffectorTriggeredLong() || operatorDashboard.ignoreEndEffectorBeamBreak.get()
//                        // scheduling the trajectory wastes a cycle; instead, reset the superstructure and run the trajectory at the same time
//                        Commands.parallel(
//                                // TODO dont need this or wait?
//                                superstructure.cancel(),
//                                drive.followTrajectory(next.scoreTraj)
//                        )
//                ));

                last = next;
            }
        }

//        last.scoreTraj.atTime("score").onTrue(CommandsExt.eagerSequence(
//                last.scoreCommand(superstructure),
//                Commands.runOnce(() -> ref.isFinished = true)
//        ));

        return startCmd;
    }

    public record IntakeScorePair(
            Trajectory<SwerveSample> stationTraj,
            StationAlign.Station station,
            Trajectory<SwerveSample> scoreTraj,
            ReefAlign.ReefZoneSide reefZoneSide,
            ReefAlign.LocalReefSide localReefSide,
            OperatorDashboard.CoralScoringLevel coralScoringLevel
    ) {
        private Command scoreCommand(Superstructure superstructure) {
            return superstructure.autoScoreCoral(
                    () -> reefZoneSide,
                    () -> localReefSide,
                    () -> coralScoringLevel,
                    () -> true
            );
        }
    }
}
