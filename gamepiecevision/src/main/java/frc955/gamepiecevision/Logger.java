package frc955.gamepiecevision;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.text.DateFormat;
import java.util.Date;
import java.util.HashSet;

public class Logger {
    private static final DataLogWriter writer;

    // are we just grabbing robot pose from nt?
    // so then how do we get the robot pose then
    // couldn't we just do the field to cam after sending the data to rio?
    // Pose2d fieldToCamera = robotPose.toPose2d().transformBy(robotToCameraTransform2d);
    // I was just thinking that we could get the cluster relative to the camera
    // then after the input is processed to be adjusted to be field to cam
    // would that not work?
    // yeah basically. Because the main thing causing the loop times to spike is the clustering right?
    // similar to how april tag vision works.
    // photon sends the data relative to the camera,
    // it has no idea where the camera is relative to the robot

    // how would I compute the transform tho? just look at robot state? in the actual code?
    // which transform
    // ohhhhhhh that makes sense
    // no, the code should be fully robot relative
    // that way there's no back and forth and on rio we just transform from robot pose to robot to fuel/cluster
    // wdym field to cam
    // like we need the robot to cam for trig to work
    // yeah idk what the 6328 code does but I think this is best:
    // 1. compute Transform3d of robot to fuel/cluster
    // well this is on orange pi so we dont know robt pose
    // so we just assume the camera is at 0,0,0? and then transform based on that?
    // yeah ig so
    // i havent thought about it as 0,0,0, just that everything is relative to the camera but yeah its the same thing ig
    // maybe? idk tbh, if justin joins he can help
    // ill s pam him
    // so just cam transform? like the one in constants?
    // lol
    // 2. send transform over nt
    // 3. on rio, robotPose.transformBy(cluster) = fieldtoCluster
    // so the orange pi doesn't do anything with cam to robot?
    // yes, but how do you do the trig without cam to robot
    // apriltag uses a 3d solve, you can't do that with gpv
    // if you didnt use 3d mode in photon vision, you cant do that, you need to adjust for robot ca

    static {
        try {
            writer = new DataLogWriter("logs/" + DateFormat.getDateInstance().format(new Date()) + ".wpilog");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static DataLog getLog() {
        return writer;
    }

    private static final String timestampKey = "/Timestamp";
    private static int timestampID;

    public static void periodicBeforeUserCode() {
        timestampID = writer.start(timestampKey, "int64", "", 0);
    }
}
