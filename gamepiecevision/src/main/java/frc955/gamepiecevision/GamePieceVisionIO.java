package frc955.gamepiecevision;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.struct.Struct;
import org.littletonrobotics.junction.RecordStruct;

public class GamePieceVisionIO {
    public static class GamePieceVisionIOInputs {
        public boolean connected = false;
        public TargetObservation[] targetObservations = new TargetObservation[0];

        public final NTPublishers ntPublishers = new NTPublishers();

        public class NTPublishers {
            private final BooleanPublisher connectedPublisher = NetworkTableInstance.getDefault()
                    .getBooleanTopic("GamePieceVision/Inputs/Connected").publish();
            private final StructArrayPublisher<TargetObservation> targetObservationsPublisher = NetworkTableInstance.getDefault()
                    .getStructArrayTopic("GamePieceVision/Inputs/TargetObservations", TargetObservation.struct).publish();

            public void publish() {
                connectedPublisher.set(connected);
                targetObservationsPublisher.set(targetObservations);
            }
        }

        public final DataLogEntries dataLogEntries = new DataLogEntries();

        public class DataLogEntries {
            private final BooleanLogEntry connectedEntry = new BooleanLogEntry(Logger.getLog(), "Inputs/Connected");
            private final StructArrayLogEntry<TargetObservation> targetObservationsEntry = StructArrayLogEntry.create(Logger.getLog(), "Inputs/TargetObservations", TargetObservation.struct);

            public void append() {
                connectedEntry.append(connected, Logger.getTimestamp());
                targetObservationsEntry.append(targetObservations, Logger.getTimestamp());
            }
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