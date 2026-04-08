package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.lib.Util;
import frc.robot.RobotState;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Optional;

public class AutoManager {
    private static final RobotState robotState = RobotState.get();

    public static final double READY_THRESHOLD_METERS = 0.1;
    public static final double MAX_DISTANCE_METERS = 3.0;
    public static final double READY_THRESHOLD_RADIANS = Math.toRadians(5);
    public static final double MAX_ROTATION_ERROR_RADIANS = Math.toRadians(180);

    private final LoggedDashboardChooser<Auto> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

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

        autoChooser.addOption("None", null);
        //autoChooser.addOption("LeftSideAuto", new LeftSideAuto());
        //autoChooser.addOption("RightSideAuto", new RightSideAuto());
        autoChooser.addOption("Canadian Depot", new CanadianDepotAuto());
        autoChooser.addOption("Orbit at the outpost", new OrbitAtTheOutpostAuto());
        autoChooser.addOption("Orbit at Home Depot", new OrbitAtHomeDepotAuto());
        autoChooser.addOption("Aura", new AuraAuto());
        autoChooser.addOption("AuraOutpost", new AuraAutoOutpost());
        autoChooser.addOption("AuraDepot", new AuraAutoDepot());
        autoChooser.addOption("Test", new OrbitGoingToHomeDepotAuto());
        autoChooser.addOption("Canadian Depot Intake", new CanadianDepotIntakeAuto());
        autoChooser.addOption("Canadian Outpost", new CanadianOutpostAuto());
        //autoChooser.addOption("CanadianJn Outpost", new CanadianOutpostJAuto());
        autoChooser.addOption("Orbit at the depot 2nd pass", new OrbitAtDepotSecondAuto());
        autoChooser.addOption("Orbit at the outpost 2nd pass", new OrbitAtOutpostSecondAuto());
        //autoChooser.addOption("CandiantOutpostAutolessAgro", new CanadianOutpostAutolessAgro());
        autoChooser.addOption("I want a turret (Orbit passing at the outpost)", new OrbitPassingOutpostAuto());

        robotState.setAutoStartPoseSupplier(this::getSelectedAutoStartingPose);

    }

    public Command getSelectedAutoCommand() {
        Auto selectedAuto = autoChooser.get();

        if (selectedAuto == null) {
            return Commands.none();
        }

        return selectedAuto.command;
    }

    public Optional<Pose2d> getSelectedAutoStartingPose() {
        Auto selectedAuto = autoChooser.get();

        if (selectedAuto == null) {
            return Optional.empty();
        }

        return Optional.of(AllianceFlipUtil.apply(selectedAuto.startingPose));
    }

    private Optional<Double> getDistanceToAutoStart() {
        return getSelectedAutoStartingPose()
                .map(startPose -> robotState.getPose().getTranslation()
                        .getDistance(startPose.getTranslation()));
    }

    private Optional<Double> getRotationErrorToAutoStart() {
        return getSelectedAutoStartingPose()
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
