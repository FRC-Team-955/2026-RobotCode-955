package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.*;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.*;

public class GamePieceVision implements Periodic {
    private static final RobotState robotState = RobotState.get();

    private final EnumMap<Camera, CameraData> cameras =
            Util.createEnumMap(Camera.class,
                    Camera.values(), (cam) -> new CameraData(
                            new GamePieceVisionIOInputsAutoLogged(),
                            cam.createIO(),
                            new Alert("Game piece vision camera " + cam.name() + " is disconnected.", Alert.AlertType.kError)
                    ));

    private final Map<Translation2d, Double> targetsToLastSeen = new HashMap<>();
    @Getter
    private List<Translation2d> bestTargets = List.of();

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
        for (Map.Entry<Camera, CameraData> cam : cameras.entrySet()) {
            Camera metadata = cam.getKey();
            CameraData data = cam.getValue();
            data.io.updateInputs(data.inputs);
            Logger.processInputs("Inputs/GamePieceVision/" + metadata.name(), data.inputs);
            // Update disconnected alert
            data.disconnectedAlert.set(!data.inputs.connected);
        }

        Map<Translation2d, Double> newlySeenTargets = new HashMap<>();
        List<Translation2d> targetXYPoints = new LinkedList<>();

        for (Map.Entry<Camera, CameraData> cam : cameras.entrySet()) {
            Camera metadata = cam.getKey();
            CameraData data = cam.getValue();

            // Process observations
            for (var observation : data.inputs.targetObservations) {
                var robotPose2d = robotState.getPoseAtTimestamp(observation.timestampSeconds());
                if (robotPose2d.isEmpty()) {
                    continue;
                }
                var robotPose = new Pose3d(robotPose2d.get());

                // Based on https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/ObjectDetection.java#L105
                // Copyright (c) 2025-2026 Littleton Robotics
                // http://github.com/Mechanical-Advantage
                Translation2d pitchYawTranslation =
                        new Translation2d(Math.tan(observation.yawRad() * txtyMultiplier.get()), Math.tan(-observation.pitchRad() * txtyMultiplier.get()))
                                .rotateBy(new Rotation2d(-metadata.robotToCamera.getRotation().getX()));
                targetXYPoints.add(pitchYawTranslation);
                double targetYaw = Math.atan(pitchYawTranslation.getX());
                double targetPitch = -Math.atan(pitchYawTranslation.getY());

                double cameraToFuelNorm =
                        (-metadata.robotToCamera.getZ() + (FieldConstants.fuelDiameter / 2))
                                / Math.tan(-metadata.robotToCamera.getRotation().getY() + targetPitch)
                                / Math.cos(targetYaw);

                Pose2d robotToCameraPose2d = new Pose3d(metadata.robotToCamera.getTranslation(), metadata.robotToCamera.getRotation()).toPose2d();
                Transform2d robotToCameraTransform2d = new Transform2d(robotToCameraPose2d.getTranslation(), robotToCameraPose2d.getRotation());

                Pose2d fieldToCamera = robotPose.toPose2d().transformBy(robotToCameraTransform2d);

                Pose2d fieldToFuel =
                        fieldToCamera
                                .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-targetYaw)))
                                .transformBy(new Transform2d(new Translation2d(cameraToFuelNorm, 0), Rotation2d.kZero));
                Pose3d fuelPose = new Pose3d(fieldToFuel).plus(new Transform3d(0, 0, FieldConstants.fuelDiameter / 2.0, new Rotation3d()));

                newlySeenTargets.put(
                        fuelPose.getTranslation().toTranslation2d(),
                        observation.timestampSeconds()
                );
            }
        }

        // Handle newly seen targets
        for (var target : newlySeenTargets.keySet()) {
            // Remove old targets within distance to be counted as the same target
            targetsToLastSeen.keySet()
                    .removeIf(otherTarget -> target.getDistance(otherTarget) < minDistanceForSameCoralMeters);
        }
        targetsToLastSeen.putAll(newlySeenTargets);

        // Remove expired coral
        targetsToLastSeen.values().removeIf(lastSeen -> Timer.getTimestamp() - lastSeen > expireTimeSeconds);

        List<List<Translation2d>> clusters = new ArrayList<>();

        for (Translation2d fuel : targetsToLastSeen.keySet()) {
            boolean addedToCluster = false;

            for (List<Translation2d> cluster : clusters) {
                for (Translation2d clusterFuel : cluster) {
                    if (fuel.getDistance(clusterFuel) < clusterGroupingDistanceMeters) {
                        cluster.add(fuel);
                        addedToCluster = true;
                        break;
                    }
                }
                if (addedToCluster) {
                    break;
                }
            }

            if (!addedToCluster) {
                List<Translation2d> newCluster = new ArrayList<>();
                newCluster.add(fuel);
                clusters.add(newCluster);
            }
        }

        bestTargets = clusters.stream()
                .max(Comparator.comparing(List::size))
                .orElse(List.of())
                .stream()
                .sorted(Comparator.comparingDouble(t -> t.getDistance(robotState.getTranslation())))
                .toList();

        Logger.recordOutput("GamePieceVision/TargetXYPoints", targetXYPoints.toArray(Translation2d[]::new));
        Logger.recordOutput("GamePieceVision/BestTargets", bestTargets.toArray(Translation2d[]::new));
    }

    @Override
    public void periodicAfterCommands() {

        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput(
                "GamePieceVision/CameraPoses",
                Arrays.stream(Camera.values())
                        .map(cam -> robotPose.transformBy(cam.robotToCamera))
                        .toArray(Pose3d[]::new)
        );
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
}

