package frc955.gamepiecevision.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.struct.Struct;

public class LoggedStructArray<T> {
    private final StructArrayPublisher<T> publisher;
    private final StructArrayLogEntry<T> entry;

    public LoggedStructArray(String name, Struct<T> struct) {
        publisher = NetworkTableInstance.getDefault().getStructArrayTopic(Logger.addPrefix(name), struct).publish();
        entry = StructArrayLogEntry.create(Logger.getLog(), Logger.addPrefix(name), struct);
    }

    public void set(T... value) {
        publisher.set(value);
        entry.append(value, Logger.getTimestamp());
    }
}
