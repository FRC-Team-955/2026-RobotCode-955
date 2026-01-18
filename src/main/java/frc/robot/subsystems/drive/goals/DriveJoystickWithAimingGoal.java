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
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveTuning.headingOverrideGainsTunable;

@RequiredArgsConstructor
public class DriveJoystickWithAimingGoal extends DriveGoal {
    private static final LoggedTunableNumber maxAccelerationMetersPerSecSquared = new LoggedTunableNumber("Drive/DriveJoystickWithAiming/MaxAccelerationMetersPerSecSquared", 5);

    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private final SlewRateLimiter2d linearAccelerationLimiter = new SlewRateLimiter2d(maxAccelerationMetersPerSecSquared.get(), robotState.getMeasuredChassisSpeeds());
    private final PIDController headingOverride = headingOverrideGainsTunable.getOrOriginal().toPIDWrapRadians();

    private Rotation2d lastLinearDirection = new Rotation2d();

    @Override
    public DriveRequest getRequest() {
        headingOverrideGainsTunable.ifChanged(gains -> gains.applyPID(headingOverride));
        if (maxAccelerationMetersPerSecSquared.hasChanged()) {
            linearAccelerationLimiter.setLimit(maxAccelerationMetersPerSecSquared.get());
        }

        //////////////////////////////////////////////////////////////////////

        Rotation2d linearDirection = controller.getDriveLinearDirection();
        double linearMagnitude = controller.getDriveLinearMagnitude();
        if (linearMagnitude == 0.0) {
            linearDirection = lastLinearDirection;
        } else {
            lastLinearDirection = linearDirection;
        }
        Translation2d linearVelocity = new Translation2d(linearMagnitude, linearDirection);
        linearVelocity = linearAccelerationLimiter.calculate(linearVelocity);
        Logger.recordOutput("Drive/DriveJoystickWithAiming/LinearMagnitudeLimited", linearVelocity.getNorm());

        return DriveRequest.chassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                linearVelocity.getX(),
                linearVelocity.getY(),
                headingOverride.calculate(robotState.getRotation().getRadians(), shootingKinematics.getShootingParameters().headingRad()),
                robotState.getRotation()
        ));
    }
}
