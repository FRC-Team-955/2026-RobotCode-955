package frc955.gamepiecevision;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.struct.Struct;

import java.nio.ByteBuffer;

public record Result(
        boolean connected,
        double timestamp,
        Transform2d[] clusters
) {
    public static final int maxClusters = 4;

    public static Struct<Result> struct = new Struct<>() {
        @Override
        public Class<Result> getTypeClass() {
            return Result.class;
        }

        @Override
        public String getTypeName() {
            return "Result";
        }

        @Override
        public int getSize() {
            return kSizeBool + kSizeDouble + Transform2d.struct.getSize() * maxClusters + kSizeInt32;
        }

        @Override
        public String getSchema() {
            return "bool connected;double timestamp;Transform2d clusters[" + maxClusters + "];int32 clustersLength";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct[]{Transform2d.struct};
        }

        @Override
        public Result unpack(ByteBuffer bb) {
            boolean connected = bb.get() != 0;
            double timestamp = bb.getDouble();

            Transform2d[] rawClusters = Struct.unpackArray(bb, maxClusters, Transform2d.struct);
            int clustersLength = bb.getInt();
            Transform2d[] clusters = new Transform2d[clustersLength];
            System.arraycopy(
                    rawClusters, 0,
                    clusters, 0,
                    clustersLength
            );

            return new Result(connected, timestamp, clusters);
        }

        @Override
        public void pack(ByteBuffer bb, Result value) {
            //if (value.clusters().length > maxClusters) {
            //    DriverStation.reportWarning("Too many clusters found to pack", false);
            //}

            bb.put(value.connected() ? (byte) 1 : (byte) 0);
            bb.putDouble(value.timestamp());

            for (int i = 0; i < maxClusters; i++) {
                if (i < value.clusters().length) {
                    Transform2d.struct.pack(bb, value.clusters()[i]);
                } else {
                    Transform2d.struct.pack(bb, new Transform2d());
                }
            }
            bb.putInt(Math.min(maxClusters, value.clusters().length));
        }
    };
}
