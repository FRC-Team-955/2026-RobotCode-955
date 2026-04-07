package frc955.gamepiecevision.logging;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.IntegerLogEntry;

public class LoggedInteger {
    private final IntegerPublisher publisher;
    private final IntegerLogEntry entry;

    /** name MUST start with "/" */
    public LoggedInteger(String name) {
        publisher = NetworkTableInstance.getDefault()
                .getIntegerTopic("/GamePieceVision/" + name).publish();
        entry = new IntegerLogEntry(Logger.getLog(), name);
    }

    public void set(long value) {
        publisher.set(value);
        entry.append(value, Logger.getTimestamp());
    }
}
