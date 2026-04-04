package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.Alert;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.Camera;

public class GamePieceVision implements Periodic {
    private static final RobotState robotState = RobotState.get();

    private final EnumMap<Camera, CameraData> cameras =
            Util.createEnumMap(Camera.class,
                    Camera.values(), (cam) -> new CameraData(
                            new GamePieceVisionIOInputsAutoLogged(),
                            cam.createIO(),
                            new Alert("Game piece vision camera " + cam.name() + " is disconnected.", Alert.AlertType.kError)
                    ));

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


    StructSubscriber<Transform2d> bestTargetSubscriber = NetworkTableInstance.getDefault()
            .getStructTopic("GamePieceVision/BestTarget", Transform2d.struct).subscribe(new Transform2d());
    DoubleSubscriber timestampSecondsSubscriber = NetworkTableInstance.getDefault()
            .getDoubleTopic("GamePieceVision/timestampSeconds").subscribe(0.0);

    @Override
    public void periodicAfterCommands() {

        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput(
                "GamePieceVision/CameraPoses",
                Arrays.stream(Camera.values())
                        .map(cam -> robotPose.transformBy(cam.robotToCamera))
                        .toArray(Pose3d[]::new)
        );
        Logger.recordOutput("GamePieceVision/RealBestTarget", robotState.getPoseAtTimestamp(
                timestampSecondsSubscriber.get()).orElse(new Pose2d()).transformBy(bestTargetSubscriber.get()));
    }

    public boolean anyCamerasDisconnected() {
        for (Map.Entry<Camera, CameraData> cam : cameras.entrySet()) {
            CameraData data = cam.getValue();
            return !data.inputs.connected;
        }
        return false;
    }


    private record CameraData(
            GamePieceVisionIOInputsAutoLogged inputs,
            GamePieceVisionIO io,
            Alert disconnectedAlert
    ) {
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

