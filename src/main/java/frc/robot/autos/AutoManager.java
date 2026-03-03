package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AllianceFlipUtil;
import frc.lib.Util;
import frc.lib.commands.CommandsExt;
import frc.robot.RobotState;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Optional;

public class AutoManager {
    private static final RobotState robotState = RobotState.get();
    public final LoggedDashboardChooser<Auto> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    public static final double READY_THRESHOLD_METERS = 0.1;
    public static final double MAX_DISTANCE_METERS = 3.0;
    public static final double READY_THRESHOLD_RADIANS = Math.toRadians(5);
    public static final double MAX_ROTATION_ERROR_RADIANS = Math.toRadians(180);

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

        //autoChooser.addOption("None", Commands.none());
        //autoChooser.addOption("LeftSideAuto", LeftSideAuto.build());
        //autoChooser.addOption("RightSideAuto", RightSideAuto.build());
        //autoChooser.addOption("DepotAuto", DepotAuto.build());
        //autoChooser.addOption("Orbit at Home Depot", OrbitAtHomeDepotAuto.build());
        //autoChooser.addOption("PlanetaryRightAuto", PlanetaryRightAuto.build());
        //autoChooser.addOption("CenterAuto", CenterAuto.build());
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

        Pose2d leftStartPose = LeftSideAuto.getStartingPose();
        Pose2d rightStartPose = RightSideAuto.getStartingPose();
        Pose2d OrbitAtHomeDepotStartPose = OrbitAtHomeDepotAuto.getStartingPose();
        Pose2d planetaryRightStartPose = PlanetaryRightAuto.getStartingPose();
        Pose2d centerStartPose = CenterAuto.getStartingPose();

        double distanceToLeft = currentPose.getTranslation().getDistance(leftStartPose.getTranslation());
        double distanceToRight = currentPose.getTranslation().getDistance(rightStartPose.getTranslation());
        double distanceToOrbitAtHomeDepot = currentPose.getTranslation().getDistance(OrbitAtHomeDepotStartPose.getTranslation());
        double distanceToPlanetaryRight = currentPose.getTranslation().getDistance(planetaryRightStartPose.getTranslation());
        double distanceToCenter = currentPose.getTranslation().getDistance(centerStartPose.getTranslation());

        double minDistance = Math.min(distanceToLeft,
                Math.min(distanceToRight,
                        Math.min(distanceToOrbitAtHomeDepot,
                                Math.min(distanceToPlanetaryRight, distanceToCenter))));

        if (minDistance == distanceToLeft) {
            return Optional.of(leftStartPose);
        } else if (minDistance == distanceToRight) {
            return Optional.of(rightStartPose);
        } else if (minDistance == distanceToOrbitAtHomeDepot) {
            return Optional.of(OrbitAtHomeDepotStartPose);
        } else if (minDistance == distanceToPlanetaryRight) {
            return Optional.of(planetaryRightStartPose);
        } else {
            return Optional.of(centerStartPose);
        }
    }

    public Command getClosestAuto() {
        Pose2d currentPose = robotState.getPose();

        Pose2d leftStartPose = LeftSideAuto.getStartingPose();
        Pose2d rightStartPose = RightSideAuto.getStartingPose();

        double distanceToLeft = currentPose.getTranslation().getDistance(leftStartPose.getTranslation());
        double distanceToRight = currentPose.getTranslation().getDistance(rightStartPose.getTranslation());

        if (distanceToLeft <= distanceToRight) {
            return LeftSideAuto.build();
        } else {
            return RightSideAuto.build();
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
