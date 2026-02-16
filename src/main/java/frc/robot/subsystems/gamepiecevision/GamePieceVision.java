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

    private final Map<Pose3d, Double> coralPoseToLastSeen = new HashMap<>();
    @Getter
    private List<Pose3d> freshCoral = List.of();
    @Getter
    private List<Pose3d> staleCoral = List.of();

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
        Map<Pose3d, Double> newlySeenCoral = new HashMap<>();
        List<Translation2d> targetPoints = new LinkedList<>();

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
                targetPoints.add(targetYawPitch);

                // Calculate distance using: theta = gamepieceWidthInPixels / pixelToRad
                //                           distance = GamepieceDiameter / tan(theta)
                double theta = observation.gamePieceWidthInPixels() / pixelsToRad;
                double distance = fuelDiameterMeters / Math.tan(theta);

                Logger.recordOutput("GamePieceVision/theta", theta);
                Logger.recordOutput("GamePieceVision/distance", distance);

                // CamToGamepiece = Vector(Distance, Heading + Cam_yaw + AngleToGamepiece)
                // We need the camera's horizontal facing direction in field coordinates

                // Get the camera's rotation in field coordinates
                Pose3d camPoseField = robotPose.transformBy(metadata.robotToCamera);

                // To find the camera's horizontal facing direction, we need to find where
                // the camera's forward axis (+X in the transformed frame) points in the XY plane
                // Create a unit vector pointing forward in camera space and transform it
                Translation3d forwardInCam = new Translation3d(1, 0, 0);
                // Rotate this vector by the camera's field rotation to get the direction it's pointing
                Rotation3d camRotationField = camPoseField.getRotation();
                Translation3d forwardInField = forwardInCam.rotateBy(camRotationField);

                // Project onto XY plane and get the angle
                double camHeadingField = Math.atan2(forwardInField.getY(), forwardInField.getX());

                // PhotonVision yaw: positive = target is to the left of center
                // Our convention: positive angle = counter-clockwise
                // So we need to negate the yaw to convert from PhotonVision convention
                double angleToGamepiece = -observation.yawRad();
                double totalAngle = camHeadingField + angleToGamepiece;

                Logger.recordOutput("GamePieceVision/camHeadingField", Math.toDegrees(camHeadingField));
                Logger.recordOutput("GamePieceVision/angleToGamepiece", Math.toDegrees(angleToGamepiece));
                Logger.recordOutput("GamePieceVision/totalAngle", Math.toDegrees(totalAngle));

                Translation2d camToGamepiece = new Translation2d(distance, new Rotation2d(totalAngle));

                // GamepiecePose = RobotPose + RobotToCam + CamToGamepiece
                // Get camera position in field coordinates
                Translation2d camPositionField = camPoseField.toPose2d().getTranslation();

                // Add camToGamepiece vector to get final gamepiece position
                Translation2d gamepiecePosition = camPositionField.plus(camToGamepiece);
                Pose3d gamepiecePose = new Pose3d(gamepiecePosition.getX(), gamepiecePosition.getY(), fuelDiameterMeters / 2.0, new Rotation3d());

                Logger.recordOutput("GamePieceVision/camToGamepieceX", camToGamepiece.getX());
                Logger.recordOutput("GamePieceVision/camToGamepieceY", camToGamepiece.getY());

                // TODO: kalman filter or PoseEstimator for stability?
                newlySeenCoral.put(gamepiecePose, observation.timestampSeconds());
            }
        }

        // Handle newly seen coral
        for (var pose : newlySeenCoral.keySet()) {
            // Remove old coral within distance to be counted as the same piece of coral
            coralPoseToLastSeen.keySet()
                    .removeIf(otherPose -> pose.getTranslation().getDistance(otherPose.getTranslation()) < minDistanceForSameCoralMeters);
        }
        coralPoseToLastSeen.putAll(newlySeenCoral);

        // Remove expired coral
        coralPoseToLastSeen.values().removeIf(lastSeen -> Timer.getTimestamp() - lastSeen > staleExpireTimeSeconds);

        // Generate fresh/stale arrays
        freshCoral = new LinkedList<>();
        staleCoral = new LinkedList<>();
        for (var entry : coralPoseToLastSeen.entrySet()) {
            Pose3d pose = entry.getKey();
            double lastSeen = entry.getValue();

            if (Timer.getTimestamp() - lastSeen < freshExpireTimeSeconds) {
                freshCoral.add(pose);
            } else {
                staleCoral.add(pose);
            }
        }

        // Log results
        Logger.recordOutput("GamePieceVision/TargetPoints", targetPoints.toArray(Translation2d[]::new));
        Logger.recordOutput("GamePieceVision/FreshCoral", freshCoral.toArray(Pose3d[]::new));
        Logger.recordOutput("GamePieceVision/StaleCoral", staleCoral.toArray(Pose3d[]::new));
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
