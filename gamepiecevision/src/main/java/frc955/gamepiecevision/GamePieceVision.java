package frc955.gamepiecevision;

import edu.wpi.first.math.geometry.*;
import frc955.gamepiecevision.GamePieceVisionIO.GamePieceVisionIOInputs;
import frc955.gamepiecevision.GamePieceVisionIO.GamePieceVisionIOInputsLogger;
import frc955.gamepiecevision.logging.LoggedDouble;
import frc955.gamepiecevision.logging.LoggedInteger;
import frc955.gamepiecevision.logging.LoggedStruct;
import frc955.gamepiecevision.logging.LoggedStructArray;
import frc955.gamepiecevision.multiobjecttracking.DBSCAN;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import static frc955.gamepiecevision.SharedGamePieceVisionConstants.fuelDiameterMeters;
import static frc955.gamepiecevision.SharedGamePieceVisionConstants.robotToCamera;

public class GamePieceVision {
    private final GamePieceVisionIOInputs inputs = new GamePieceVisionIOInputs();
    private final GamePieceVisionIOInputsLogger inputsLogger = new GamePieceVisionIOInputsLogger();
    private final GamePieceVisionIO io = new GamePieceVisionIOPhotonVision("IntakeCam");

    private final LoggedStruct<Result> loggedResult = new LoggedStruct<>("Outputs/Result", Result.struct);
    private final LoggedDouble loggedBestWeight = new LoggedDouble("Outputs/BestWeight");
    private final LoggedInteger loggedNumClusters = new LoggedInteger("Outputs/NumClusters");
    private final LoggedInteger loggedLargestCluster = new LoggedInteger("Outputs/LargestCluster");
    private final LoggedStructArray<Translation2d> loggedTargetPoints = new LoggedStructArray<>("Outputs/TargetPoints", Translation2d.struct);
    private final LoggedStructArray<Translation2d> loggedAllTargets = new LoggedStructArray<>("Outputs/AllTargets", Translation2d.struct);

    private final DBSCAN dbscan = new DBSCAN(new ArrayList<>(), 3, 0.3);

    /** length should be the same as Result.maxClusters */
    private static final Double[] wantedDistances = new Double[]{null, 2.0, 1.5, 1.0};
    private final Translation2d[] lastTargets = new Translation2d[Result.maxClusters];

    public void periodic() {
        io.updateInputs(inputs);
        inputsLogger.log(inputs);

        List<Translation2d> targetPointsOnCapture = new LinkedList<>();
        List<Translation2d> targetsOnField = new LinkedList<>();

        // Process observations
        for (var observation : inputs.targetObservations) {
            // Based on https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/ObjectDetection.java#L105
            // Copyright (c) 2025-2026 Littleton Robotics
            // http://github.com/Mechanical-Advantage
            Translation2d pitchYawTranslation =
                    new Translation2d(Math.tan(observation.yawRad()), Math.tan(-observation.pitchRad()))
                            .rotateBy(new Rotation2d(-robotToCamera.getRotation().getX()));
            targetPointsOnCapture.add(pitchYawTranslation);
            double targetYaw = Math.atan(pitchYawTranslation.getX());
            double targetPitch = -Math.atan(pitchYawTranslation.getY());

            double cameraToFuelNorm =
                    (-robotToCamera.getZ() + (fuelDiameterMeters / 2))
                            / Math.tan(-robotToCamera.getRotation().getY() + targetPitch)
                            / Math.cos(targetYaw);

            Pose2d robotToCameraPose2d = new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation()).toPose2d();
            Pose2d robotToFuel =
                    robotToCameraPose2d
                            .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-targetYaw)))
                            .transformBy(new Transform2d(new Translation2d(cameraToFuelNorm, 0), Rotation2d.kZero));
            //Pose3d fuelPose = new Pose3d(robotToFuel).plus(new Transform3d(0, 0, fuelDiameterMeters / 2.0, new Rotation3d()));

            if (robotToFuel.getTranslation().getNorm() > 3.0) {
                continue;
            }

            targetsOnField.add(robotToFuel.getTranslation());
        }

        List<Transform2d> clusters = new LinkedList<>();
        for (int i = 0; i < wantedDistances.length; i++) {
            Double wantedDistance = wantedDistances[i];
            List<Translation2d> filteredTargetsOnField = wantedDistance != null
                    ? targetsOnField.stream().filter(t -> t.getNorm() < wantedDistance).toList()
                    : targetsOnField;
            Translation2d cluster = findBestCluster(i, filteredTargetsOnField);
            if (cluster != null) {
                clusters.add(new Transform2d(cluster.getX(), cluster.getY(), Rotation2d.kZero));
            }
        }

        loggedResult.set(new Result(
                inputs.connected,
                inputs.timestamp,
                clusters.toArray(Transform2d[]::new)
        ));
        loggedTargetPoints.set(targetPointsOnCapture.toArray(Translation2d[]::new));
        loggedAllTargets.set(targetsOnField.toArray(Translation2d[]::new));
    }

    private Translation2d findBestCluster(int index, List<Translation2d> filteredTargetsOnField) {
        Translation2d bestTarget = null;
        double bestWeight = 0;
        int largestSize = 0;

        ArrayList<ArrayList<Translation2d>> dbscanResults = new ArrayList<>();
        if (filteredTargetsOnField.size() > 2) {
            dbscan.setInputValues(filteredTargetsOnField);
            dbscanResults = dbscan.performClustering();
        }
        if (dbscanResults.isEmpty()) {
            for (Translation2d target : filteredTargetsOnField) {
                ArrayList<Translation2d> singleTarget = new ArrayList<>();
                singleTarget.add(target);
                dbscanResults.add(singleTarget);
            }
        }

        for (ArrayList<Translation2d> cluster : dbscanResults) {
            FuelCluster fuelCluster = new FuelCluster(cluster);
            double weight = fuelCluster.weight();
            if (lastTargets[index] != null && fuelCluster.avgLocation().isPresent()) {
                weight += 2 / (fuelCluster.avgLocation().get().getDistance(lastTargets[index]) + 1);
            }
            if (weight > bestWeight) {
                bestWeight = weight;
                bestTarget = fuelCluster.avgLocation().get();
            }
            if (fuelCluster.size() > largestSize) {
                largestSize = fuelCluster.size();
            }
        }

        if (index == wantedDistances.length - 1) {
            loggedBestWeight.set(bestWeight);
            loggedNumClusters.set(dbscanResults.size());
            loggedLargestCluster.set(largestSize);
        }

        lastTargets[index] = bestTarget;
        return bestTarget;
    }

    private record FuelCluster(List<Translation2d> cluster) {
        public void addFuel(Translation2d fuel) {
            cluster.add(fuel);
        }

        public int size() {
            return cluster.size();
        }

        public boolean addIfWithin(Translation2d fuel, double maxDist) {
            for (Translation2d clusterFuel : cluster) {
                if (fuel.getDistance(clusterFuel) < maxDist) {
                    cluster.add(fuel);
                    return true;
                }
            }
            return false;
        }

        public double weight() {
            if (avgLocation().isPresent()) {
                return ((double) size()) / (avgLocation().get().getNorm() + 1);
            } else {
                return 0;
            }
        }

        public Optional<Translation2d> avgLocation() {
            if (cluster.isEmpty()) {
                return Optional.empty();
            }
            Translation2d average = new Translation2d();
            for (Translation2d fuel : cluster) {
                average = average.plus(fuel);
            }
            return Optional.of(average.div(cluster.size()));
        }
    }
}
