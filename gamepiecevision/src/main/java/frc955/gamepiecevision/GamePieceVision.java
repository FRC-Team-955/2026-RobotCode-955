package frc955.gamepiecevision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.geometry.struct.Translation2dStruct;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.Timer;
import frc955.gamepiecevision.GamePieceVisionIO.GamePieceVisionIOInputs;
import frc955.gamepiecevision.multiobjecttracking.DBSCAN;

import java.util.*;

import static frc955.gamepiecevision.GamePieceVisionConstants.*;

public class GamePieceVision {
    private final GamePieceVisionIOInputs inputs = new GamePieceVisionIOInputs();
    private final GamePieceVisionIO io = new GamePieceVisionIOPhotonVision("IntakeCam");

    private final StructArrayLogEntry<Translation2d> clusterTranslationsEntry = StructArrayLogEntry.create(Logger.getLog(), "Test", Translation2d.struct);
    private final StructPublisher<Transform2d> bestTargetPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("GamePieceVision/BestTarget", Transform2d.struct).publish();
    private final DoublePublisher timestampSecondsPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("GamePieceVision/timestampSeconds").publish();

    private final Map<Translation2d, Double> targetsToLastSeen = new HashMap<>();

    private final DBSCAN dbscan = new DBSCAN(new ArrayList<Translation2d>(), 3, 0.5);

    private Optional<Translation2d> lastTarget = Optional.empty();

    public void periodic() {
        io.updateInputs(inputs);
        inputs.ntPublishers.publish();
        inputs.dataLogEntries.append();

        Map<Translation2d, Double> newlySeenTargets = new HashMap<>();
        List<Translation2d> targetXYPoints = new LinkedList<>();

        // Process observations
        for (var observation : inputs.targetObservations) {
            // Based on https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/ObjectDetection.java#L105
            // Copyright (c) 2025-2026 Littleton Robotics
            // http://github.com/Mechanical-Advantage
            Translation2d pitchYawTranslation =
                    new Translation2d(Math.tan(observation.yawRad()), Math.tan(-observation.pitchRad()))
                            .rotateBy(new Rotation2d(-robotToCamera.getRotation().getX()));
            targetXYPoints.add(pitchYawTranslation);
            double targetYaw = Math.atan(pitchYawTranslation.getX());
            double targetPitch = -Math.atan(pitchYawTranslation.getY());

            double cameraToFuelNorm =
                    (-robotToCamera.getZ() + (fuelDiameterMeters / 2))
                            / Math.tan(-robotToCamera.getRotation().getY() + targetPitch)
                            / Math.cos(targetYaw);

            Pose2d robotToCameraPose2d = new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation()).toPose2d();
            Transform2d robotToCameraTransform2d = new Transform2d(robotToCameraPose2d.getTranslation(), robotToCameraPose2d.getRotation());

            // 1. compute Transform3d of robot to fuel/cluster
            // 2. send transform over nt
            // 3. on rio, robotPose.transformBy(cluster) = fieldtoCluster

            // this part needs to get fixed since we don't have robot pos

            // TODO: Apply in actual robot code
            // Pose2d fieldToCamera = robotPose.toPose2d().transformBy(robotToCameraTransform2d);

            Pose2d robotToFuel =
                    robotToCameraPose2d
                            .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-targetYaw)))
                            .transformBy(new Transform2d(new Translation2d(cameraToFuelNorm, 0), Rotation2d.kZero));
            Pose3d fuelPose = new Pose3d(robotToFuel).plus(new Transform3d(0, 0, fuelDiameterMeters / 2.0, new Rotation3d()));

            if (fuelPose.getTranslation().getNorm() > 3.0) {
                continue;
            }

            newlySeenTargets.put(
                    fuelPose.getTranslation().toTranslation2d(),
                    observation.timestampSeconds()
            );
            timestampSecondsPublisher.set(observation.timestampSeconds());

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

        ArrayList<FuelCluster> clusterList = new ArrayList<FuelCluster>();
        Translation2d bestTarget = null;
        double bestWeight = 0;
        int largestSize = 0;

        ArrayList<ArrayList<Translation2d>> dbscanResults = new ArrayList<ArrayList<Translation2d>>();
        if (targetsToLastSeen.size() > 2) {
            dbscan.setInputValues(targetsToLastSeen.keySet());
            dbscanResults = dbscan.performClustering();
        }
        if (dbscanResults.isEmpty()) {
            for (Translation2d target : targetsToLastSeen.keySet()) {
                ArrayList<Translation2d> singleTarget = new ArrayList<Translation2d>();
                singleTarget.add(target);
                dbscanResults.add(singleTarget);
            }
        }

        for (ArrayList<Translation2d> cluster : dbscanResults) {
            FuelCluster fuelCluster = new FuelCluster(cluster);
            clusterList.add(fuelCluster);
            double weight = fuelCluster.weight();
            if (lastTarget.isPresent() && fuelCluster.avgLocation().isPresent()) {
                weight += 2 / (fuelCluster.avgLocation().get().getDistance(lastTarget.get()) + 1);
            }
            if (weight > bestWeight) {
                bestWeight = weight;
                bestTarget = fuelCluster.avgLocation().get();
            }
            if (fuelCluster.size() > largestSize) {
                largestSize = fuelCluster.size();
            }
        }
        if (bestTarget != null) {
            bestTargetPublisher.set(new Transform2d(bestTarget.getX(), bestTarget.getY(), Rotation2d.kZero));
        }
        lastTarget = Optional.ofNullable(bestTarget);
        /*
        Logger.recordOutput("GamePieceVision/BestCluster", bestTarget);
        Logger.recordOutput("GamePieceVision/BestWeight", bestWeight);
        Logger.recordOutput("GamePieceVision/NumClusters", dbscanResults.size());
        Logger.recordOutput("GamePieceVision/LargestCluster", largestSize);
         */
        //bestTargets = clusters
        //        .stream()
        //        .sorted(Comparator.comparing(FuelCluster::size))
        //        .map(FuelCluster::avgLocation)
        //        .flatMap(Optional::stream)
        //        .toList();
        //bestTargets = clusters
        //        .stream()
        //        .max(Comparator.comparing(FuelCluster::size))
        //        .map(FuelCluster::avgLocation)
        //        .stream()
        //        .toList();
        //bestTargets = targetsToLastSeen
        //        .keySet()
        //        .stream()
        //        .sorted(Comparator.comparingDouble(t -> t.getDistance(robotState.getTranslation())))
        //        .toList();

        //Logger.recordOutput("GamePieceVision/TargetXYPoints", targetXYPoints.toArray(Translation2d[]::new));
        //Logger.recordOutput("GamePieceVision/AllTargets", targetsToLastSeen.keySet().toArray(Translation2d[]::new));
        //Logger.recordOutput("GamePieceVision/BestTargets", bestTargets.toArray(Translation2d[]::new));
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

