package frc955.gamepiecevision.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;

public class LoggedStruct<T> {
    private final StructPublisher<T> publisher;
    private final StructLogEntry<T> entry;

    public LoggedStruct(String name, Struct<T> struct) {
        publisher = NetworkTableInstance.getDefault().getStructTopic(Logger.addPrefix(name), struct).publish();
        entry = StructLogEntry.create(Logger.getLog(), Logger.addPrefix(name), struct);
    }

    public void set(T value) {
        publisher.set(value);
        entry.append(value, Logger.getTimestamp());
    }
}
