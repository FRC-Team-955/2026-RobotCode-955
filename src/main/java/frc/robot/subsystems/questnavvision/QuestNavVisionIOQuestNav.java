package frc.robot.subsystems.questnavvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import java.util.Arrays;

public class QuestNavVisionIOQuestNav extends QuestNavVisionIO {
    private static final int batteryLowPercent = 20;
    private static final int batteryCriticalPercent = 10;

    private final QuestNav questNav = new QuestNav();

    public QuestNavVisionIOQuestNav() {
        questNav.setVersionCheckEnabled(false);

        questNav.onConnected(() -> System.out.println("Quest connected"));
        questNav.onDisconnected(() -> DriverStation.reportWarning("Quest disconnected", false));
        questNav.onTrackingAcquired(() -> System.out.println("Quest tracking acquired"));
        questNav.onTrackingLost(() -> DriverStation.reportWarning("Quest tracking lost", false));
        questNav.onLowBattery(batteryLowPercent, level ->
                DriverStation.reportWarning("Quest battery low: " + level + "%", false));
        questNav.onCommandSuccess(response ->
                System.out.println("Quest pose reset succeeded: " + response.getCommandId()));
        questNav.onCommandFailure(response ->
                DriverStation.reportError("Quest pose reset failed: " + response.getErrorMessage(), false));
    }

    @Override
    public void updateInputs(QuestNavVisionIOInputs inputs) {
        questNav.commandPeriodic();

        inputs.connected = questNav.isConnected();
        inputs.tracking = questNav.isTracking();
        inputs.latency = questNav.getLatency();
        // QuestNav provides an integer percentage via OptionalInt; use -1 for unknown
        inputs.batteryPercent = questNav.getBatteryPercent().orElse(-1);

        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
        inputs.unreadFrameCount = frames.length;
        inputs.poseObservations = Arrays.stream(frames)
                .map(frame -> new PoseObservation(
                        frame.dataTimestamp(),
                        frame.questPose3d(),
                        frame.isTracking()))
                .toArray(PoseObservation[]::new);

        if (inputs.batteryPercent >= 0 && inputs.batteryPercent < batteryCriticalPercent) {
            DriverStation.reportWarning("Quest battery CRITICAL: " + inputs.batteryPercent + "%", false);
        }
    }

    @Override
    public void resetPose(Pose3d questPose) {
        questNav.setPose(questPose);
    }
}


