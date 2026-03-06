package frc.robot.subsystems.gamepiecevision.multiobjecttracking;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;

/**
 * Implementation of density-based clustering algorithm DBSCAN.
 *
 * Original Publication:
 * Ester, Martin; Kriegel, Hans-Peter; Sander, Jörg; Xu, Xiaowei (1996).
 * Simoudis, Evangelos; Han, Jiawei; Fayyad, Usama M., eds.
 * A density-based algorithm for discovering clusters in large spatial
 * databases with noise. Proceedings of the Second International Conference
 * on Knowledge Discovery and Data Mining (KDD-96). AAAI Press. pp. 226-231
 *
 * Usage:
 * - Identify type of input values.
 * - Implement metric for input value type using DistanceMetric interface.
 * - Invoke {@link #performClustering()}.
 *
 * See tests and metrics for example implementation and use.
 *
 * @author <a href="mailto:cf@christopherfrantz.org">Christopher Frantz</a>
 * @version 0.1
 */
public class DBSCAN {

    /** maximum distance of values to be considered as cluster */
    private double epsilon = 1f;

    /** minimum number of members to consider cluster */
    private int minimumNumberOfClusterMembers = 2;

    /** internal list of input values to be clustered */
    private ArrayList<Translation2d> inputValues = null;

    /** index maintaining visited points */
    private HashSet<Translation2d> visitedPoints = new HashSet<Translation2d>();

    /**
     * Creates a DBSCAN clusterer instance.
     * Upon instantiation, call {@link #performClustering()}
     * to perform the actual clustering.
     *
     * @param inputValues Input values to be clustered
     * @param minNumElements Minimum number of elements to constitute cluster
     * @param maxDistance Maximum distance of elements to consider clustered
     */
    public DBSCAN(final Collection<Translation2d> inputValues, int minNumElements, double maxDistance) {
        setInputValues(inputValues);
        setMinimalNumberOfMembersForCluster(minNumElements);
        setMaximalDistanceOfClusterMembers(maxDistance);
    }

    /**
     * Sets a collection of input values to be clustered.
     * Repeated calls overwrite the original input values.
     *
     * @param collection Input values to be clustered
     */
    public void setInputValues(final Collection<Translation2d> collection) {
        this.inputValues = new ArrayList<Translation2d>(collection);
    }

    /**
     * Sets the minimal number of members to consider points of close proximity
     * clustered.
     *
     * @param minimalNumberOfMembers Minimum number of members to constitute a cluster
     */
    public void setMinimalNumberOfMembersForCluster(final int minimalNumberOfMembers) {
        this.minimumNumberOfClusterMembers = minimalNumberOfMembers;
    }

    /**
     * Sets the maximal distance members of the same cluster can have while
     * still be considered in the same cluster.
     *
     * @param maximalDistance Maximum distance between individual members
     */
    public void setMaximalDistanceOfClusterMembers(final double maximalDistance) {
        this.epsilon = maximalDistance;
    }

    /**
     * Determines the neighbours of a given input value.
     *
     * @param inputValue Input value for which neighbours are to be determined
     * @return List of neighbours for a given input value
     */
    private ArrayList<Translation2d> getNeighbours(final Translation2d inputValue) {
        ArrayList<Translation2d> neighbours = new ArrayList<Translation2d>();
        for(int i=0; i<inputValues.size(); i++) {
            Translation2d candidate = inputValues.get(i);
            if (inputValue.getDistance(candidate) <= epsilon) {
                neighbours.add(candidate);
            }
        }
        return neighbours;
    }

    /**
     * Merges the elements of the right collection to the left one and returns
     * the combination.
     *
     * @param neighbours1 left collection
     * @param neighbours2 right collection
     * @return Modified left collection
     */
    private ArrayList<Translation2d> mergeRightToLeftCollection(final ArrayList<Translation2d> neighbours1,
                                                    final ArrayList<Translation2d> neighbours2) {
        for (int i = 0; i < neighbours2.size(); i++) {
            Translation2d tempPt = neighbours2.get(i);
            if (!neighbours1.contains(tempPt)) {
                neighbours1.add(tempPt);
            }
        }
        return neighbours1;
    }

    /**
     * Applies the clustering and returns a collection of clusters (i.e., a list
     * of lists of the respective cluster members).
     *
     * @return Collection of clusters identified as part of the clustering process
     */
    public ArrayList<ArrayList<Translation2d>> performClustering() {
        ArrayList<ArrayList<Translation2d>> resultList = new ArrayList<ArrayList<Translation2d>>();
        visitedPoints.clear();

        ArrayList<Translation2d> neighbours;
        int index = 0;

        while (inputValues.size() > index) {
            Translation2d p = inputValues.get(index);
            if (!visitedPoints.contains(p)) {
                visitedPoints.add(p);
                neighbours = getNeighbours(p);

                if (neighbours.size() >= minimumNumberOfClusterMembers) {
                    int ind = 0;
                    while (neighbours.size() > ind) {
                        Translation2d r = neighbours.get(ind);
                        if (!visitedPoints.contains(r)) {
                            visitedPoints.add(r);
                            ArrayList<Translation2d> individualNeighbours = getNeighbours(r);
                            if (individualNeighbours.size() >= minimumNumberOfClusterMembers) {
                                neighbours = mergeRightToLeftCollection(
                                        neighbours,
                                        individualNeighbours);
                            }
                        }
                        ind++;
                    }
                    resultList.add(neighbours);
                }
            }
            index++;
        }
        return resultList;
    }

}
