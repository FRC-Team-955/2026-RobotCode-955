package frc955.gamepiecevision;

import edu.wpi.first.util.struct.Struct;
import frc955.gamepiecevision.logging.LoggedBoolean;
import frc955.gamepiecevision.logging.LoggedStructArray;
import org.littletonrobotics.junction.RecordStruct;

public class GamePieceVisionIO {
    public static class GamePieceVisionIOInputs {
        public boolean connected = false;
        public TargetObservation[] targetObservations = new TargetObservation[0];
    }

    public static class GamePieceVisionIOInputsLogger {
        private final LoggedBoolean connected = new LoggedBoolean("Inputs/Connected");
        private final LoggedStructArray<TargetObservation> targetObservations = new LoggedStructArray<>("Inputs/TargetObservations", TargetObservation.struct);

        public void log(GamePieceVisionIOInputs inputs) {
            connected.set(inputs.connected);
            targetObservations.set(inputs.targetObservations);
        }
    }

    public record TargetObservation(
            double timestampSeconds,
            // yaw and pitch are not necessarily rotations, but
            // correspond to a point on a 2d plane
            double yawRad,
            double pitchRad
    ) {
        @SuppressWarnings("unchecked")
        public static Struct<TargetObservation> struct = (Struct<TargetObservation>) new RecordStruct(TargetObservation.class);
    }

    public void updateInputs(GamePieceVisionIOInputs inputs) {
    }
}