package frc.robot.subsystems.questnavvision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public class QuestNavVisionIO {
    @AutoLog
    public static class QuestNavVisionIOInputs {
        public boolean connected = false;
        public boolean tracking = false;
        public double latency = 0.0;
        // battery percent reported as integer percentage, -1 = unknown
        public int batteryPercent = -1;
        public int unreadFrameCount = 0;
        public PoseObservation[] poseObservations = new PoseObservation[0];
    }

    public record PoseObservation(
            double timestamp,
            Pose3d questPose,
            boolean tracking
    ) {
    }

    public void updateInputs(QuestNavVisionIOInputs inputs) {
    }

    public void resetPose(Pose3d questPose) {
    }
}


