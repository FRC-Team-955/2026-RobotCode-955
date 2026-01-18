package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.PIDF;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.Controller;
import frc.robot.RobotState;
import frc.robot.ShootingKinematics;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;
import static frc.robot.subsystems.drive.DriveTuning.headingOverrideGainsTunable;

@RequiredArgsConstructor
public class DriveJoystickWithHeadingOverrideGoal extends DriveGoal {
    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private final PIDController headingOverride = headingOverrideGainsTunable.getOrOriginal().toPIDWrapRadians();

    @Override
    public DriveRequest getRequest() {
        headingOverrideGainsTunable.ifChanged(gains -> gains.applyPID(headingOverride));

        //////////////////////////////////////////////////////////////////////

        ChassisSpeeds joystickSetpoint = controller.getDriveSetpointRobotRelative(robotState.getRotation());

        joystickSetpoint = new ChassisSpeeds(
                joystickSetpoint.vxMetersPerSecond,
                joystickSetpoint.vyMetersPerSecond,
                headingOverride.calculate(robotState.getRotation().getRadians(), shootingKinematics.getShootingParameters().headingRad())
        );

        return DriveRequest.chassisSpeeds(joystickSetpoint);
    }
}
