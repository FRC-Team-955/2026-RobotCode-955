package frc955.gamepiecevision;

public class GamePieceVisionIO {
    public static class GamePieceVisionIOInputs {
        public boolean connected = false;
        public TargetObservation[] targetObservations = new TargetObservation[0];
    }

    public record TargetObservation(
            double timestampSeconds,
            // yaw and pitch are not necessarily rotations, but
            // correspond to a point on a 2d plane
            double yawRad,
            double pitchRad
    ) {}

    public void updateInputs(GamePieceVisionIOInputs inputs) {
    }
}