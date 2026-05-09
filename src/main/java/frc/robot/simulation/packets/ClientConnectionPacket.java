package frc.robot.simulation.packets;

import java.nio.ByteBuffer;

/** Sent by clients to server */
public record ClientConnectionPacket(int id) {
    public static final int SIZE = Integer.BYTES;

    private static final ByteBuffer packBuf = ByteBuffer.allocate(SIZE);

    public byte[] toBytes() {
        packBuf.clear();

        packBuf.putInt(id);

        return packBuf.array();
    }

    public static ClientConnectionPacket fromBytes(byte[] rawUnpackBuf, int length) {
        var unpackBuf = ByteBuffer.wrap(rawUnpackBuf, 0, length);

        int id = unpackBuf.getInt();

        return new ClientConnectionPacket(id);
    }
}
