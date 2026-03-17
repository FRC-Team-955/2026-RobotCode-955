package frc.robot.subsystems.gamepiecevision.multiobjecttracking;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

public class MultiFuelTracking {
    public static final double CREATION_DISTANCE_METERS = 0.5;
    public static final double DELETION_DISTANCE_METERS = 0.5;

    private final HashMap<Integer, FuelObservation> fuelObservations = new HashMap<Integer, FuelObservation>();

    //public Translation2d calculate(List<FuelCluster> clusters) {}

    private double[][] distanceMatrix(List<FuelObservation> currentObservations, List<Translation2d> newObservations) {
        int currentSize = currentObservations.size();
        int newSize = newObservations.size();
        int totalSize = currentSize + newSize;

        // In this matrix, rows = current observations, columns = new observations
        // This forms a 2x2 square block matrix, upper left is distances, upper right is the weight required to mark a ball as disappeared,
        // Bottom left is the weight required to create a new observation
        // Bottom right is zero
        double[][] distanceMatrix = new double[totalSize][totalSize];

        // Top left
        for (int i = 0; i < currentSize; i++) {
            for (int j = 0; j < newSize; j++) {
                // This can be edited, currently we are only going off of euclidean distance
                distanceMatrix[i][j] = currentObservations.get(i).getPose().getDistance(newObservations.get(j));
            }
        }

        // Bottom left
        for (int i = currentSize; i < totalSize; i++) {
            for (int j = 0; j < newSize; j++) {
                distanceMatrix[i][j] = CREATION_DISTANCE_METERS;
            }
        }

        // Top right
        for (int i = 0; i < currentSize; i++) {
            for (int j = newSize; j < totalSize; j++) {
                distanceMatrix[i][j] = DELETION_DISTANCE_METERS;
            }
        }

        return distanceMatrix;
    }

    public static class FuelObservation {
        private final List<Translation2d> recentPoses = new ArrayList<>();
        private final static int MAX_STORAGE_SIZE = 5;
        private final static int DECAY_TIME_FRAMES = 5;
        private static int ID = 0;
        private final int id;
        private double confidence = 1;

        public FuelObservation(Translation2d pose) {
            id = ID;
            ID++;
            recentPoses.add(pose);
        }

        // Add pose, return if the ball has disappeared
        public boolean addPose(Optional<Translation2d> pose) {
            if (pose.isPresent()) {
                if (recentPoses.size() > MAX_STORAGE_SIZE) {
                    recentPoses.remove(0);
                }
                recentPoses.add(pose.get());
                confidence = 1;
            } else {
                confidence -= (double) 1 / DECAY_TIME_FRAMES;
            }

            return confidence > 0;
        }

        public Translation2d getPose() {
            Translation2d avg = new Translation2d();
            for (Translation2d pose : recentPoses) {
                avg = avg.plus(pose);
            }
            return avg.div(recentPoses.size());
        }

        public int getID() {
            return id;
        }
    }

    public static class FuelCluster {
        private final List<Translation2d> cluster = new ArrayList<>();

        public void addFuel(Translation2d fuel) {
            cluster.add(fuel);
        }

        public int size() {
            return cluster.size();
        }

        public double weight() {
            return ((double) size()) / avgLocation().getNorm();
        }

        public Translation2d avgLocation() {
            Translation2d average = new Translation2d();
            for (Translation2d fuel : cluster) {
                average = average.plus(fuel);
            }
            return average.div(cluster.size());
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
    }
}
