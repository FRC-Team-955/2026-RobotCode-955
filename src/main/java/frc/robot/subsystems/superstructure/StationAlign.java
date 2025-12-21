package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.lib.AllianceBasedPose2d;
import frc.lib.Util;
import frc.robot.RobotState;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class StationAlign {
    private static final RobotState robotState = RobotState.get();

    public static final double alignLinearToleranceMeters = 0.05;
    public static final double alignAngularToleranceRad = Units.degreesToRadians(10);

    private static final Transform2d alignOffsetBargeSide = new Transform2d(0, 0.6, Rotation2d.k180deg);
    private static final Transform2d alignOffsetProcessorSide = new Transform2d(
            alignOffsetBargeSide.getX(),
            -alignOffsetBargeSide.getY(),
            alignOffsetBargeSide.getRotation()
    );
    private static final Transform2d alignOffsetProcessorSideFriendly = new Transform2d(
            alignOffsetProcessorSide.getX(),
            alignOffsetProcessorSide.getY() + 1.15,
            alignOffsetProcessorSide.getRotation()
    );

    @RequiredArgsConstructor
    public enum Station {
        BargeSide(StationAlign.getAlignPose(1, alignOffsetBargeSide)),
        ProcessorSide(StationAlign.getAlignPose(0, alignOffsetProcessorSide)),
        ProcessorSideFriendly(StationAlign.getAlignPose(0, alignOffsetProcessorSideFriendly));

        private final AllianceBasedPose2d alignPose;

        public Pose2d getAlignPose() {
            return alignPose.get();
        }
    }

    private static AllianceBasedPose2d getAlignPose(int aprilTagOffset, Transform2d alignOffset) {
        return new AllianceBasedPose2d(
                // blue
                // 12 = processor side, 13 = barge side
                AlignHelpers.getAprilTagPose(12 + (aprilTagOffset % 2)).plus(AlignHelpers.bumperOffset).plus(alignOffset),

                // red
                // 2 = processor side, 1 = barge side
                AlignHelpers.getAprilTagPose(1 + ((1 - aprilTagOffset) % 2)).plus(AlignHelpers.bumperOffset).plus(alignOffset)
        );
    }

    public static boolean atAlignPose(Station station) {
        Pose2d align = station.getAlignPose();
        boolean positionMet = robotState.isAtPoseWithTolerance(align, alignLinearToleranceMeters, alignAngularToleranceRad);
        Logger.recordOutput("Superstructure/StationAlign/PositionMet", positionMet);
        return positionMet;
    }
}
