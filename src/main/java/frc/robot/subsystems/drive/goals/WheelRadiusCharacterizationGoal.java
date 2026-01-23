package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.drivebaseRadiusMeters;

@RequiredArgsConstructor
public class WheelRadiusCharacterizationGoal extends DriveGoal {
    private static final LoggedTunableNumber speedRadPerSec = new LoggedTunableNumber("Drive/WheelRadiusCharacterization/SpeedRadPerSec", 1.0);

    private static final Drive drive = Drive.get();

    @RequiredArgsConstructor
    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int value;
    }

    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);
    private final double[] startWheelPositions = drive.getWheelRadiusCharacterizationPositions();

    private double lastGyroYawRad = drive.getRawGyroRotation().getRadians();
    private double accumGyroYawRad = 0.0;

    @Override
    public DriveRequest getRequest() {
        var omega = omegaLimiter.calculate(omegaDirection.value * speedRadPerSec.get());

        // Get yaw and wheel positions
        accumGyroYawRad += MathUtil.angleModulus(drive.getRawGyroRotation().getRadians() - lastGyroYawRad);
        lastGyroYawRad = drive.getRawGyroRotation().getRadians();
        double averageWheelPosition = 0.0;
        double[] wheelPositions = drive.getWheelRadiusCharacterizationPositions();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        double currentEffectiveWheelRadius = (accumGyroYawRad * drivebaseRadiusMeters) / averageWheelPosition;
        Logger.recordOutput("Drive/WheelRadiusCharacterization/DrivePosition", averageWheelPosition);
        Logger.recordOutput("Drive/WheelRadiusCharacterization/AccumGyroYawRad", accumGyroYawRad);
        Logger.recordOutput("Drive/WheelRadiusCharacterization/CurrentWheelRadiusInches", Units.metersToInches(currentEffectiveWheelRadius));
        Logger.recordOutput("Drive/WheelRadiusCharacterization/HasEnoughData", Math.abs(accumGyroYawRad) > Math.PI * 2.0);

        return DriveRequest.chassisSpeeds(new ChassisSpeeds(0, 0, omega));
    }
}
