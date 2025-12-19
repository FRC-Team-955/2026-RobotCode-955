package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;

import java.util.function.Supplier;

@RequiredArgsConstructor
public class VelocityRobotRelativeGoal extends DriveGoal {
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    @Override
    public DriveRequest getRequest() {
        return DriveRequest.chassisSpeedsOptimized(chassisSpeedsSupplier.get());
    }
}