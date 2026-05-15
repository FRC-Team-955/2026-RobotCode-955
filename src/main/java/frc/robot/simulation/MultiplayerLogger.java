package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.simulation.packets.ServerUpdatePacket;

import java.util.Arrays;

public class MultiplayerLogger {
    private static final StringPublisher statusPublisher = NetworkTableInstance.getDefault().getStringTopic("/Multiplayer/Status").publish();

    public static void logStatus(String status) {
        statusPublisher.set(status + " " + ((System.currentTimeMillis() / 1000.0) % 10000.0), RobotController.getFPGATime());
    }

    private static final IntegerPublisher lastTrySendPublisher = NetworkTableInstance.getDefault().getIntegerTopic("/Multiplayer/LastTrySend").publish();

    public static void logTrySend() {
        lastTrySendPublisher.set(System.currentTimeMillis(), RobotController.getFPGATime());
    }

    private static final IntegerPublisher lastSendPublisher = NetworkTableInstance.getDefault().getIntegerTopic("/Multiplayer/LastSend").publish();

    public static void logSend() {
        lastSendPublisher.set(System.currentTimeMillis(), RobotController.getFPGATime());
    }

    private static final IntegerPublisher lastTryReceivePublisher = NetworkTableInstance.getDefault().getIntegerTopic("/Multiplayer/LastTryReceive").publish();

    public static void logTryReceive() {
        lastTryReceivePublisher.set(System.currentTimeMillis(), RobotController.getFPGATime());
    }

    private static final IntegerPublisher lastReceivePublisher = NetworkTableInstance.getDefault().getIntegerTopic("/Multiplayer/LastReceive").publish();

    public static void logReceive() {
        lastReceivePublisher.set(System.currentTimeMillis(), RobotController.getFPGATime());
    }

    private static final IntegerArrayPublisher connectedIdsPublisher = NetworkTableInstance.getDefault().getIntegerArrayTopic("/Multiplayer/ConnectedIDs").publish();
    private static final StructArrayPublisher<Pose2d> robotPosesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Multiplayer/RobotPoses", Pose2d.struct).publish();
    private static final StructArrayPublisher<Translation3d> fuelTranslationsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/Multiplayer/FuelTranslations", Translation3d.struct).publish();

    private static final Field2d field2d = new Field2d();
    private static final boolean useField2d = true;

    public static void logUpdate(ServerUpdatePacket update) {
        // Messy but this is what AdvantageKit does
        long[] asLong = new long[update.connectedIds().length];
        for (int i = 0; i < update.connectedIds().length; i++) {
            asLong[i] = update.connectedIds()[i];
        }
        connectedIdsPublisher.set(asLong, RobotController.getFPGATime());

        robotPosesPublisher.set(update.robotPoses(), RobotController.getFPGATime());

        fuelTranslationsPublisher.set(update.fuelTranslations(), RobotController.getFPGATime());

        if (useField2d) {
            if (update.robotPoses().length > 0) {
                field2d.setRobotPose(update.robotPoses()[0]);
                for (int i = 1; i < update.robotPoses().length; i++) {
                    field2d.getObject("Robot" + i).setPose(update.robotPoses()[i]);
                }
            }
            field2d.getObject("Fuel").setPoses(Arrays.stream(update.fuelTranslations())
                    .map(t -> new Pose2d(t.toTranslation2d(), new Rotation2d()))
                    .toList()
            );
            SmartDashboard.putData("Multiplayer/Field", field2d);
        }
    }
}
