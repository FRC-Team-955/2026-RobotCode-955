package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.SlewRateLimiter2d;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.Controller;
import frc.robot.RobotState;
import frc.robot.ShootingKinematics;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;

import static frc.robot.subsystems.drive.DriveConstants.headingOverrideGains;


@RequiredArgsConstructor
public class DriveJoystickWithAimingGoal extends DriveGoal {
    private static final LoggedTunableNumber maxVelocityMetersPerSec = new LoggedTunableNumber("Drive/DriveJoystickWithAiming/MaxVelocityMetersPerSec", 2);
    private static final LoggedTunableNumber maxLinearAccelerationMetersPerSecPerSec = new LoggedTunableNumber("Drive/DriveJoystickWithAiming/MaxLinearAccelerationMetersPerSecPerSec", 5);

    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private final SlewRateLimiter2d linearAccelerationLimiter = new SlewRateLimiter2d(maxLinearAccelerationMetersPerSecPerSec.get(), robotState.getMeasuredChassisSpeeds());
    private final PIDController headingOverride = headingOverrideGains.toPIDWrapRadians();

    private Rotation2d lastLinearDirection = new Rotation2d();

    @Override
    public DriveRequest getRequest() {
        if (headingOverrideGains.hasChanged()) {
            headingOverrideGains.applyPID(headingOverride);
        }
        if (maxLinearAccelerationMetersPerSecPerSec.hasChanged()) {
            linearAccelerationLimiter.setLimit(maxLinearAccelerationMetersPerSecPerSec.get());
        }

        //////////////////////////////////////////////////////////////////////

        Rotation2d linearDirection = controller.getDriveLinearDirection();
        double linearMagnitude = controller.getDriveLinearMagnitude();
        if (linearMagnitude == 0.0) {
            linearDirection = lastLinearDirection;
        } else {
            lastLinearDirection = linearDirection;
        }

        // Use our own max velocity
        linearMagnitude *= maxVelocityMetersPerSec.get();

        Translation2d linearVelocity = new Translation2d(linearMagnitude, linearDirection);
        linearVelocity = linearAccelerationLimiter.calculate(linearVelocity);
//        Logger.recordOutput("Drive/DriveJoystickWithAiming/LinearMagnitudeLimited", linearVelocity.getNorm());

        double angularVelocity = headingOverride.calculate(robotState.getRotation().getRadians(), shootingKinematics.getShootingParameters().headingRad());

        if (linearMagnitude == 0.0 && Math.abs(angularVelocity) <= DriveConstants.joystickDriveDeadband) {
            return DriveRequest.stopWithX();
        } else {
            return DriveRequest.chassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX(),
                    linearVelocity.getY(),
                    angularVelocity,
                    robotState.getRotation()
            ));
        }
    }
}
