package frc.robot.autos;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util;
import frc.robot.RobotState;

import java.util.List;
import java.util.Optional;

public class AutoHelper {
    private static final RobotState robotState = RobotState.get();

    private static final Choreo.TrajectoryCache trajectoryCache = new Choreo.TrajectoryCache();

    @SuppressWarnings("unchecked")
    public static Trajectory<SwerveSample> trajectory(String name) {
        // Basically copied from AutoFactory
        Optional<? extends Trajectory<?>> optTrajectory = trajectoryCache.loadTrajectory(name);
        if (optTrajectory.isPresent()) {
            return (Trajectory<SwerveSample>) optTrajectory.get();
        } else {
            Util.error("Trajectory " + name + " is not present");
            //noinspection Convert2Diamond
            return new Trajectory<SwerveSample>(name, List.of(), List.of(), List.of());
        }
    }

    @SuppressWarnings("unchecked")
    public static Trajectory<SwerveSample> trajectory(String name, final int splitIndex) {
        // Basically copied from AutoFactory
        Optional<? extends Trajectory<?>> optTrajectory = trajectoryCache.loadTrajectory(name, splitIndex);
        if (optTrajectory.isPresent()) {
            return (Trajectory<SwerveSample>) optTrajectory.get();
        } else {
            Util.error("Trajectory " + name + " is not present");
            //noinspection Convert2Diamond
            return new Trajectory<SwerveSample>(name, List.of(), List.of(), List.of());
        }
    }

    public static Command resetOdometry(Trajectory<SwerveSample> trajectory) {
        // Basically copied from AutoTrajectory

        if (trajectory.getInitialPose(Util.shouldFlip()).isEmpty()) {
            Util.error("Missing initial pose for " + trajectory.name());
        }

        return Commands.runOnce(() -> {
            var initialPoseOpt = trajectory.getInitialPose(Util.shouldFlip());
            if (initialPoseOpt.isEmpty()) {
                Util.error("Missing initial pose for " + trajectory.name());
            } else {
                robotState.setPose(initialPoseOpt.get());
            }
        });
    }
}
