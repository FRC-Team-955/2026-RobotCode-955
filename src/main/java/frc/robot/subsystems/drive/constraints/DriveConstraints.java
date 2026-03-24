package frc.robot.subsystems.drive.constraints;

import frc.lib.network.LoggedTunableNumber;
import lombok.With;
import org.jetbrains.annotations.Nullable;

@With
public record DriveConstraints(
        @Nullable LoggedTunableNumber maxLinearVelocityMetersPerSec,
        @Nullable LoggedTunableNumber maxLinearAccelerationMetersPerSecPerSec,
        @Nullable LoggedTunableNumber maxAngularVelocityRadPerSec,
        @Nullable LoggedTunableNumber maxAngularAccelerationRadPerSecPerSec
) {
}
