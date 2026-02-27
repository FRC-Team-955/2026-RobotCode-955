package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
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
                Translation2d targetYawPitch = new Translation2d(observation.yawRad(), observation.pitchRad());
                targetXYPoints.add(targetYawPitch);
                // Account for roll of camera
                // TODO: fix roll compensation - this doesn't fully work
                double gamepieceWidthRad = observation.gamePieceWidthInPixels() / pixelsToRad;
                Logger.recordOutput("GamePieceVision/gamepieceWidthRad", gamepieceWidthRad);

                targetYawPitch = targetYawPitch.rotateBy(Rotation2d.fromRadians(-metadata.robotToCamera.getRotation().getX()));
                double targetYaw = targetYawPitch.getX();
                double targetPitch = targetYawPitch.getY();

                double distance = (fuelDiameterMeters / 2.0) / Math.tan(gamepieceWidthRad / 2.0);
                double camToGamepieceZ = -metadata.robotToCamera.getZ() + (fuelDiameterMeters / 2.0);
                double middleLine = Math.sqrt(Math.pow(distance, 2.0) - Math.pow(camToGamepieceZ, 2.0));

                double camToGamepieceX = Math.cos(-targetYaw) * middleLine;
                double camToGamepieceY = middleLine * Math.sin(-targetYaw);
                Logger.recordOutput("GamePieceVision/camToGamepieceZ", camToGamepieceZ);
                Logger.recordOutput("GamePieceVision/camToGamepieceX", camToGamepieceX);

                Logger.recordOutput("GamePieceVision/camToGamepieceY", camToGamepieceY);
                Translation2d camToGamepieceXY = new Translation2d(camToGamepieceX, camToGamepieceY)
                        // Account for yaw of camera
                        // Note that we do this BEFORE translating to robot coordinates
                        .rotateBy(Rotation2d.fromRadians(metadata.robotToCamera.getRotation().getZ()));
                Translation2d robotToGamepieceXY = metadata.robotToCamera.getTranslation().toTranslation2d().plus(camToGamepieceXY);
                double robotToGamepieceZ = metadata.robotToCamera.getZ() + camToGamepieceZ;
                Translation3d robotToGamepiece = new Translation3d(robotToGamepieceXY.getX(), robotToGamepieceXY.getY(), robotToGamepieceZ);


//                double totalAngle = metadata.robotToCamera.getRotation().getZ() + robotPose.toPose2d().getRotation().getRadians() + observation.yawRad();
//                Translation2d camToGamepiece = new Translation2d(distance, new Rotation2d(totalAngle));
//                Pose3d camPose = robotPose.transformBy(metadata.robotToCamera);
//
//                //    +;;).plus(camToGamepiece))
//                Pose2d gamepiecePose = new Pose2d(camPose.toPose2d().getTranslation().plus(camToGamepiece), new Rotation2d());


                // First, calculate position of target in camera space
                double camToTargetZ = -metadata.robotToCamera.getZ() + fuelDiameterMeters / 2.0;
                // Account for pitch of camera
                double camToTargetX = camToTargetZ / Math.tan(targetPitch - metadata.robotToCamera.getRotation().getY());
                double camToTargetY = camToTargetX * Math.tan(-targetYaw);

                Translation2d camToTargetXY = new Translation2d(camToTargetX, camToTargetY)
                        // Account for yaw of camera
                        // Note that we do this BEFORE translating to robot coordinates
                        .rotateBy(Rotation2d.fromRadians(metadata.robotToCamera.getRotation().getZ()));
                // Next, translate x and y to robot coordinates
                Translation2d robotToTargetXY = metadata.robotToCamera.getTranslation().toTranslation2d().plus(camToTargetXY);
                double robotToTargetZ = metadata.robotToCamera.getZ() + camToTargetZ;

                Translation3d robotToTarget = new Translation3d(robotToTargetXY.getX(), robotToTargetXY.getY(), robotToTargetZ);
                Logger.recordOutput("GamePieceVision/tagX", camToTargetXY.getX());

                newlySeenTargets.put(
                        robotPose
                                .transformBy(new Transform3d(
                                        robotToGamepiece,
                                        new Rotation3d()
                                ))
                                .getTranslation()
                                .toTranslation2d(),
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

        // For now, just sort by distance
        // In the future, we should find the biggest group of targets
        bestTargets = targetsToLastSeen.keySet().stream().sorted((a, b) -> {
            double aDist = a.getDistance(robotState.getTranslation());
            double bDist = b.getDistance(robotState.getTranslation());
            return Double.compare(aDist, bDist);
        }).toList();

        // Log results
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
