package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.createIO;
import static frc955.gamepiecevision.SharedGamePieceVisionConstants.robotToCamera;

public class GamePieceVision implements Periodic {
    private static final RobotState robotState = RobotState.get();

    private final GamePieceVisionIOInputsAutoLogged inputs = new GamePieceVisionIOInputsAutoLogged();
    private final GamePieceVisionIO io = createIO();

    private final Alert disconnectedAlert = new Alert("Game piece vision camera is disconnected.", Alert.AlertType.kError);

    private static GamePieceVision instance;


    public static synchronized GamePieceVision get() {
        if (instance == null) {
            instance = new GamePieceVision();
        }

        return instance;
    }

    private GamePieceVision() {
        if (instance != null) {
            Util.error("Duplicate GamePieceVision created");
        }
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

        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput(
                "GamePieceVision/CameraPoses",
                robotPose.transformBy(robotToCamera)
        );
        Logger.recordOutput("GamePieceVision/BestTarget", new Pose2d(getBestTarget().orElse(new Translation2d()), new Rotation2d()));
    }

    public boolean anyCamerasDisconnected() {
        return !inputs.connected;
    }

    public Optional<Translation2d> getBestTarget() {
        if (inputs.clusters.length == 0) {
            return Optional.empty();
        }
        return robotState.getPoseAtTimestamp(inputs.timestamp)
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

