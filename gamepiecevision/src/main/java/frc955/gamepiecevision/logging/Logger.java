package frc955.gamepiecevision.logging;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.wpilibj.RobotController;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Date;

public class Logger {
    private static final DataLogWriter writer;

    private static final LoggedInteger startTimestamp;
    private static final LoggedInteger endTimestamp;

    static {
        try {
            Files.createDirectories(Path.of("logs"));
            writer = new DataLogWriter("logs/" + new Date() + ".wpilog");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        startTimestamp = new LoggedInteger("Timestamp");
        endTimestamp = new LoggedInteger("EndTimestamp");
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
        startTimestamp.set(timestamp);
    }

    public static void periodicAfterCode() {
        long timestamp = RobotController.getFPGATime();
        endTimestamp.set(timestamp);

        writer.flush();
    }
}
