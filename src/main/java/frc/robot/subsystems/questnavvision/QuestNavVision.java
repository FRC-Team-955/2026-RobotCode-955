package frc.robot.subsystems.questnavvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.LinkedList;
import java.util.List;

import static frc.robot.FieldConstants.fieldLength;
import static frc.robot.FieldConstants.fieldWidth;
import static frc.robot.subsystems.questnavvision.QuestNavVisionConstants.*;

public class QuestNavVision implements Periodic {
    private static final RobotState robotState = RobotState.get();

    private final QuestNavVisionIOInputsAutoLogged inputs = new QuestNavVisionIOInputsAutoLogged();
    private final QuestNavVisionIO io = createIO();

    private final Alert disconnectedAlert = new Alert("QuestNav is disconnected.", Alert.AlertType.kError);
    private final Alert trackingLostAlert = new Alert("QuestNav tracking is lost.", Alert.AlertType.kWarning);

    private static QuestNavVision instance;

    public static synchronized QuestNavVision get() {
        if (instance == null) {
            instance = new QuestNavVision();
        }

        return instance;
    }

    private QuestNavVision() {
        if (instance != null) {
            Util.error("Duplicate QuestNavVision created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/QuestNavVision", inputs);

        disconnectedAlert.set(!inputs.connected);
        trackingLostAlert.set(inputs.connected && !inputs.tracking);

        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> acceptedRobotPoses = new LinkedList<>();
        List<Pose3d> rejectedRobotPoses = new LinkedList<>();

        for (var observation : inputs.poseObservations) {
            Pose3d robotPose = observation.questPose().plus(robotToQuest.inverse());
            allRobotPoses.add(robotPose);

            boolean rejectPose =
                    !observation.tracking()
                            || Math.abs(robotPose.getZ()) > maxZErrorMeters
                            || robotPose.getX() < 0.0
                            || robotPose.getX() > fieldLength
                            || robotPose.getY() < 0.0
                            || robotPose.getY() > fieldWidth;

            if (rejectPose) {
                rejectedRobotPoses.add(robotPose);
                continue;
            }

            acceptedRobotPoses.add(robotPose);
            robotState.addVisionMeasurement(
                    robotPose.toPose2d(),
                    observation.timestamp(),
                    linearStdDevMeters,
                    angularStdDevRad);
        }

        Logger.recordOutput("QuestNavVision/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("QuestNavVision/RobotPosesAccepted", acceptedRobotPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("QuestNavVision/RobotPosesRejected", rejectedRobotPoses.toArray(Pose3d[]::new));
        if (!allRobotPoses.isEmpty()) {
            Logger.recordOutput("QuestNavVision/LatestRobotPose", allRobotPoses.get(allRobotPoses.size() - 1));
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("QuestNavVision/RobotPoseEstimate", new Pose3d(robotState.getPose()));
    }

    public boolean disconnected() {
        return !inputs.connected;
    }

    public Command resetPoseToRobotState() {
        return Commands.runOnce(() -> resetPose(robotState.getPose())).ignoringDisable(true);
    }

    public void resetPose(Pose2d robotPose) {
        Pose3d questPose = new Pose3d(robotPose).plus(robotToQuest);
        io.resetPose(questPose);
    }
}

