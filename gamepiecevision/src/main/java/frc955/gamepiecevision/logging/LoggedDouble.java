package frc955.gamepiecevision.logging;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public class LoggedDouble {
    private final DoublePublisher publisher;
    private final DoubleLogEntry entry;

    public LoggedDouble(String name) {
        publisher = NetworkTableInstance.getDefault().getDoubleTopic(Logger.addPrefix(name)).publish();
        entry = new DoubleLogEntry(Logger.getLog(), Logger.addPrefix(name));
    }

    public void set(double value) {
        publisher.set(value);
        entry.append(value, Logger.getTimestamp());
    }
}
