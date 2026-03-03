package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.lib.Util;
import frc.lib.commands.CommandsExt;
import frc.robot.RobotState;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;
import java.util.Optional;

public class AutoManager {
    private static final RobotState robotState = RobotState.get();
    public final LoggedDashboardChooser<Auto> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    public static final double READY_THRESHOLD_METERS = 0.1;
    public static final double MAX_DISTANCE_METERS = 3.0;
    public static final double READY_THRESHOLD_RADIANS = Math.toRadians(5);
    public static final double MAX_ROTATION_ERROR_RADIANS = Math.toRadians(180);

    private static final LeftSideAuto leftSideAuto = new LeftSideAuto();
    private static final RightSideAuto rightSideAuto = new RightSideAuto();
    private static final DepotAuto depotAuto = new DepotAuto();
    private static final OrbitAtHomeDepotAuto orbitAtHomeDepotAuto = new OrbitAtHomeDepotAuto();
    private static final PlanetaryRightAuto planetaryRightAuto = new PlanetaryRightAuto();
    private static final CenterAuto centerAuto = new CenterAuto();

    private static final List<Auto> allAutos = List.of(
            leftSideAuto,
            rightSideAuto,
            depotAuto,
            orbitAtHomeDepotAuto,
            planetaryRightAuto,
            centerAuto
    );

    private static AutoManager instance;

    public static synchronized AutoManager get() {
        if (instance == null) {
            instance = new AutoManager();
        }

        return instance;
    }

    private AutoManager() {
        if (instance != null) {
            Util.error("Duplicate AutoManager created");
        }

        //autoChooser.addOption("None", new Auto(new Pose2d(), Commands.none()));
        autoChooser.addOption("LeftSideAuto", leftSideAuto);
        autoChooser.addOption("RightSideAuto", rightSideAuto);
        autoChooser.addOption("DepotAuto", depotAuto);
        autoChooser.addOption("Orbit at Home Depot", orbitAtHomeDepotAuto);
        autoChooser.addOption("PlanetaryRightAuto", planetaryRightAuto);
        autoChooser.addOption("CenterAuto", centerAuto);
    }

    public Command getSelectedAuto() {
        Auto selectedAuto = autoChooser.get();
        return CommandsExt.eagerSequence(
                robotState.setPose(() -> AllianceFlipUtil.apply(selectedAuto.startingPose)),
                selectedAuto.command
        );
    }

    public Optional<Pose2d> getClosestAutoStartingPose() {
        Pose2d currentPose = robotState.getPose();

        Auto closestAuto = null;
        double minDistance = Double.MAX_VALUE;

        for (Auto auto : allAutos) {
            double distance = currentPose.getTranslation().getDistance(auto.startingPose.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestAuto = auto;
            }
        }

        return closestAuto != null ? Optional.of(closestAuto.startingPose) : Optional.empty();
    }

    public Command getClosestAuto() {
        Pose2d currentPose = robotState.getPose();

        Pose2d leftStartPose = leftSideAuto.startingPose;
        Pose2d rightStartPose = rightSideAuto.startingPose;

        double distanceToLeft = currentPose.getTranslation().getDistance(leftStartPose.getTranslation());
        double distanceToRight = currentPose.getTranslation().getDistance(rightStartPose.getTranslation());

        if (distanceToLeft <= distanceToRight) {
            return leftSideAuto.command;
        } else {
            return rightSideAuto.command;
        }
    }

    public Optional<Double> getDistanceToAutoStart() {
        return getClosestAutoStartingPose()
                .map(startPose -> robotState.getPose().getTranslation()
                        .getDistance(startPose.getTranslation()));
    }

    public Optional<Double> getRotationErrorToAutoStart() {
        return getClosestAutoStartingPose()
                .map(startPose -> {
                    double currentRotation = robotState.getPose().getRotation().getRadians();
                    double targetRotation = startPose.getRotation().getRadians();
                    return Math.abs(MathUtil.angleModulus(currentRotation - targetRotation));
                });
    }

    public boolean isAtAutoStartingPose() {
        Optional<Double> distance = getDistanceToAutoStart();
        Optional<Double> rotationError = getRotationErrorToAutoStart();

        if (distance.isEmpty() || rotationError.isEmpty()) {
            return false;
        }

        return distance.get() <= READY_THRESHOLD_METERS &&
                rotationError.get() <= READY_THRESHOLD_RADIANS;
    }

    public double getPlacementProgress() {
        Optional<Double> distanceOpt = getDistanceToAutoStart();
        Optional<Double> rotationErrorOpt = getRotationErrorToAutoStart();

        if (distanceOpt.isEmpty() || rotationErrorOpt.isEmpty()) {
            return 0.0;
        }

        double distance = distanceOpt.get();
        double rotationError = rotationErrorOpt.get();

        double translationProgress;
        if (distance >= MAX_DISTANCE_METERS) {
            translationProgress = 0.0;
        } else if (distance <= READY_THRESHOLD_METERS) {
            translationProgress = 1.0;
        } else {
            translationProgress = 1.0 - (distance - READY_THRESHOLD_METERS) / (MAX_DISTANCE_METERS - READY_THRESHOLD_METERS);
        }

        double rotationProgress;
        if (rotationError >= MAX_ROTATION_ERROR_RADIANS) {
            rotationProgress = 0.0;
        } else if (rotationError <= READY_THRESHOLD_RADIANS) {
            rotationProgress = 1.0;
        } else {
            rotationProgress = 1.0 - (rotationError - READY_THRESHOLD_RADIANS) / (MAX_ROTATION_ERROR_RADIANS - READY_THRESHOLD_RADIANS);
        }

        return (translationProgress * 0.7) + (rotationProgress * 0.3);
    }
}
