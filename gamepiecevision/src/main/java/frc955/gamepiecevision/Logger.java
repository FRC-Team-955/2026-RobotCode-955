package frc955.gamepiecevision;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.RobotController;

import java.io.IOException;
import java.util.Date;

public class Logger {
    private static final DataLogWriter writer;

    private static final IntegerLogEntry startTimestampEntry;
    private static final IntegerLogEntry endTimestampEntry;

    static {
        try {
            writer = new DataLogWriter("logs/" + new Date() + ".wpilog");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        startTimestampEntry = new IntegerLogEntry(writer, "/Timestamp");
        endTimestampEntry = new IntegerLogEntry(writer, "/EndTimestamp");
    }

    public static DataLog getLog() {
        return writer;
    }

    private static long timestamp;

    public static long getTimestamp() {
        return timestamp;
    }

    public static void periodicBeforeCode() {
        timestamp = RobotController.getFPGATime();
        startTimestampEntry.append(timestamp, timestamp);
    }

    public static void periodicAfterCode() {
        long endTimestamp = RobotController.getFPGATime();
        endTimestampEntry.append(endTimestamp, timestamp);

        writer.flush();
    }
}
