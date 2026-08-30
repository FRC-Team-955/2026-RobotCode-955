package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotState;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;
import java.util.stream.Stream;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.createIO;
import static frc955.gamepiecevision.SharedGamePieceVisionConstants.robotToCamera;

public class GamePieceVision implements Periodic {
    private final GamePieceVisionIOInputsAutoLogged inputs = new GamePieceVisionIOInputsAutoLogged();
    private final GamePieceVisionIO io = createIO();

    private final Alert disconnectedAlert = new Alert("Game piece vision camera is disconnected.", Alert.AlertType.kError);

    @Getter
    private static final GamePieceVision instance = new GamePieceVision();

    private GamePieceVision() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/GamePieceVision", inputs);
        // Update disconnected alert
        disconnectedAlert.set(!inputs.connected);
    }

    @Override
    public void periodicAfterCommands() {
        var robotPose = new Pose3d(RobotState.getInstance().getPose());
        Logger.recordOutput(
                "GamePieceVision/CameraPoses",
                robotPose.transformBy(robotToCamera)
        );
        Logger.recordOutput("GamePieceVision/BestTarget", new Pose2d(getBestTarget().orElse(new Translation2d()), new Rotation2d()));
        Pose2d[] allTargets = getAllTargets()
                .map(t -> new Pose2d(t, new Rotation2d()))
                .toArray(Pose2d[]::new);
        Logger.recordOutput("GamePieceVision/AllTargets", allTargets);

        RobotState.getInstance().setFuel(allTargets);
    }

    public boolean anyCamerasDisconnected() {
        return !inputs.connected;
    }

    public Stream<Translation2d> getAllTargets() {
        if (!inputs.connected || inputs.clusters.length == 0) {
            return Stream.empty();
        }
        Optional<Pose2d> poseOpt = RobotState.getInstance().getPoseAtTimestamp(inputs.timestamp);
        if (poseOpt.isEmpty()) {
            return Stream.empty();
        }
        Pose2d pose = poseOpt.get();

        return Arrays.stream(inputs.clusters)
                .map(t -> pose.transformBy(t).getTranslation());
    }

    public Optional<Translation2d> getBestTarget() {
        if (!inputs.connected || inputs.clusters.length == 0) {
            return Optional.empty();
        }
        return RobotState.getInstance().getPoseAtTimestamp(inputs.timestamp)
                .map(pose -> pose.transformBy(inputs.clusters[0]).getTranslation());
    }

    //public List<Translation2d> getBestTargetsInBounds(Optional<Bounds> bounds) {
    //    List<FuelCluster> clusters = new LinkedList<>();
    //
    //    for (Translation2d fuel : targetsToLastSeen.keySet()) {
    //        if (bounds.isPresent() && !bounds.get().contains(fuel)) {
    //            continue;
    //        }
    //
    //        if (fuel.getDistance(robotState.getTranslation()) > 2.0) {
    //            continue;
    //        }
    //
    //        boolean addedToCluster = false;
    //
    //        for (FuelCluster cluster : clusters) {
    //            if (cluster.addIfWithin(fuel, clusterGroupingDistanceMeters)) {
    //                addedToCluster = true;
    //                break;
    //            }
    //        }
    //
    //        if (!addedToCluster) {
    //            FuelCluster newCluster = new FuelCluster(new LinkedList<>());
    //            newCluster.addFuel(fuel);
    //            clusters.add(newCluster);
    //        }
    //    }
    //
    //    return clusters
    //            .stream()
    //            .sorted(Comparator.comparingDouble(FuelCluster::size))
    //            .map(c -> c.cluster)
    //            .flatMap(Collection::stream)
    //            .toList();
    //}
}

