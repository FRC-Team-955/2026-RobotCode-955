package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class AlignHelpers {
    @SuppressWarnings("OptionalGetWithoutIsPresent") // better for our code to crash than to fail silently
    public static Pose2d getAprilTagPose(int id) {
        return AprilTagVisionConstants.aprilTagLayout.getTagPose(id).get().toPose2d();
    }

    public static final Transform2d bumperOffset = new Transform2d(driveConfig.bumperLengthMeters() / 2.0, 0, new Rotation2d());
}
