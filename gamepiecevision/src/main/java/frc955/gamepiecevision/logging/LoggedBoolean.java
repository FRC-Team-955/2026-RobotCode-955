package frc955.gamepiecevision.logging;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;

public class LoggedBoolean {
    private final BooleanPublisher publisher;
    private final BooleanLogEntry entry;

    /** name MUST start with "/" */
    public LoggedBoolean(String name) {
        publisher = NetworkTableInstance.getDefault()
                .getBooleanTopic("/GamePieceVision" + name).publish();
        entry = new BooleanLogEntry(Logger.getLog(), name);
    }

    public void set(boolean value) {
        publisher.set(value);
        entry.append(value, Logger.getTimestamp());
    }
}
