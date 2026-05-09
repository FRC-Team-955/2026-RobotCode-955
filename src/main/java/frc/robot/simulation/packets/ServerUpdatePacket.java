package frc.robot.simulation.packets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.nio.ByteBuffer;

/** Sent by server to clients */
public record ServerUpdatePacket(byte[] connectedIds, Pose2d[] robotPoses, Translation3d[] fuelTranslations) {
    public static final int MAX_SIZE = 10 * 1024;

    private static final ByteBuffer packBuf = ByteBuffer.allocate(MAX_SIZE);

    public byte[] toBytes() {
        packBuf.clear();

        packBuf.putInt(connectedIds.length);
        for (byte id : connectedIds) {
            packBuf.put(id);
        }

        packBuf.putInt(robotPoses.length);
        for (Pose2d pose : robotPoses) {
            packBuf.putFloat((float) pose.getX());
            packBuf.putFloat((float) pose.getY());
            packBuf.putFloat((float) pose.getRotation().getRadians());
        }

        packBuf.putInt(fuelTranslations.length);
        for (Translation3d translation : fuelTranslations) {
            packBuf.putFloat((float) translation.getX());
            packBuf.putFloat((float) translation.getY());
            packBuf.putFloat((float) translation.getZ());
        }

        return packBuf.array();
    }

    public static ServerUpdatePacket fromBytes(byte[] rawUnpackBuf, int length) {
        var unpackBuf = ByteBuffer.wrap(rawUnpackBuf, 0, length);

        int connectedIdsLength = unpackBuf.getInt();
        byte[] connectedIds = new byte[connectedIdsLength];
        for (int i = 0; i < connectedIdsLength; i++) {
            connectedIds[i] = unpackBuf.get();
        }

        int robotPosesLength = unpackBuf.getInt();
        Pose2d[] robotPoses = new Pose2d[robotPosesLength];
        for (int i = 0; i < robotPosesLength; i++) {
            double x = unpackBuf.getFloat();
            double y = unpackBuf.getFloat();
            double rot = unpackBuf.getFloat();
            robotPoses[i] = new Pose2d(x, y, Rotation2d.fromRadians(rot));
        }

        int fuelTranslationsLength = unpackBuf.getInt();
        Translation3d[] fuelTranslations = new Translation3d[fuelTranslationsLength];
        for (int i = 0; i < fuelTranslationsLength; i++) {
            double x = unpackBuf.getFloat();
            double y = unpackBuf.getFloat();
            double z = unpackBuf.getFloat();
            fuelTranslations[i] = new Translation3d(x, y, z);
        }

        return new ServerUpdatePacket(connectedIds, robotPoses, fuelTranslations);
    }
}
