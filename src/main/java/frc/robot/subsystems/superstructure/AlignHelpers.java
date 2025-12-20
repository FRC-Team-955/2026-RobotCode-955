package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.lib.Util;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class AlignHelpers {
    public static Pose2d getAprilTagPose(int id) {
        var tagPoseOpt = AprilTagVisionConstants.aprilTagLayout.getTagPose(id);
        if (tagPoseOpt.isEmpty()) {
            Util.error("Could not find pose for AprilTag with ID " + id);
            return new Pose2d();
        }
        return tagPoseOpt.get().toPose2d();
    }

    public static final Transform2d bumperOffset = new Transform2d(driveConfig.bumperLengthMeters() / 2.0, 0, new Rotation2d());
}
