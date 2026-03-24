package frc.robot.subsystems.drive.constraints;

import frc.lib.network.LoggedTunableNumber;
import lombok.With;

@With
public record DriveConstraints(
        LoggedTunableNumber maxLinearVelocityMetersPerSec,
        LoggedTunableNumber maxLinearAccelerationMetersPerSecPerSec,
        LoggedTunableNumber maxAngularVelocityRadPerSec,
        LoggedTunableNumber maxAngularAccelerationRadPerSecPerSec
) {
}
