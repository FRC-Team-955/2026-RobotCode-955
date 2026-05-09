package frc.robot.simulation.packets;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.nio.ByteBuffer;

/** Sent by clients to server */
public record ClientUpdatePacket(
        int id,
        ChassisSpeeds wantedSpeeds,
        boolean wantsToShoot,
        double flywheelRPM,
        double hoodRad
) {
    public static final int SIZE = Integer.BYTES + 3 * Float.BYTES + Byte.BYTES + Float.BYTES + Float.BYTES;

    private static final ByteBuffer packBuf = ByteBuffer.allocate(SIZE);

    public byte[] toBytes() {
        packBuf.clear();

        packBuf.putInt(id);
        packBuf.putFloat((float) wantedSpeeds.vxMetersPerSecond);
        packBuf.putFloat((float) wantedSpeeds.vyMetersPerSecond);
        packBuf.putFloat((float) wantedSpeeds.omegaRadiansPerSecond);
        packBuf.put(wantsToShoot ? (byte) 1 : (byte) 0);
        packBuf.putFloat((float) flywheelRPM);
        packBuf.putFloat((float) hoodRad);

        return packBuf.array();
    }

    public static ClientUpdatePacket fromBytes(byte[] rawUnpackBuf, int length) {
        var unpackBuf = ByteBuffer.wrap(rawUnpackBuf, 0, length);

        int id = unpackBuf.getInt();
        double wantedSpeedsVx = unpackBuf.getFloat();
        double wantedSpeedsVy = unpackBuf.getFloat();
        double wantedSpeedsOmega = unpackBuf.getFloat();
        boolean wantsToShoot = unpackBuf.get() != 0;
        double flywheelRPM = unpackBuf.getFloat();
        double hoodRad = unpackBuf.getFloat();

        return new ClientUpdatePacket(
                id,
                new ChassisSpeeds(wantedSpeedsVx, wantedSpeedsVy, wantedSpeedsOmega),
                wantsToShoot,
                flywheelRPM,
                hoodRad
        );
    }
}
