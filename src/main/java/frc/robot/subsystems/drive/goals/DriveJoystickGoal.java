package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.PIDF;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.Controller;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;
import static frc.robot.subsystems.drive.DriveTuning.headingOverrideGainsTunable;

@RequiredArgsConstructor
public class DriveJoystickGoal extends DriveGoal {
    private static final LoggedTunableNumber headingOverrideSetpointResetTime = new LoggedTunableNumber("Drive/DriveJoystick/HeadingOverrideSetpointResetTimeSeconds", 0.25);
    private static final LoggedTunableNumber headingOverrideThresholdDegrees = new LoggedTunableNumber("Drive/DriveJoystick/HeadingOverrideThresholdDegrees", 5.0);

    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();

    private final PIDController headingOverride = headingOverrideGainsTunable.getOrOriginal().toPIDWrapRadians();
    private final Timer headingOverrideSetpointResetTimer = new Timer();
    private boolean shouldRunHeadingOverride = false;

    @Override
    public DriveRequest getRequest() {
        headingOverrideGainsTunable.ifChanged(gains -> gains.applyPID(headingOverride));

        //////////////////////////////////////////////////////////////////////

        ChassisSpeeds joystickSetpoint = controller.getDriveSetpointRobotRelative(robotState.getRotation());

        if (joystickSetpoint.omegaRadiansPerSecond == 0.0) {
            // Once joystick omega is 0, wait X seconds before getting robot rotation
            // and then start heading override
            if (!shouldRunHeadingOverride) {
                if (!headingOverrideSetpointResetTimer.isRunning()) {
                    // Omega just became 0 - start timer
                    headingOverrideSetpointResetTimer.restart();
                } else if (headingOverrideSetpointResetTimer.hasElapsed(headingOverrideSetpointResetTime.get())) {
                    // Now time to set PID setpoint and start overriding heading
                    headingOverride.reset();
                    headingOverride.setSetpoint(robotState.getRotation().getRadians());
                    shouldRunHeadingOverride = true;
                }
            }

            if (shouldRunHeadingOverride) {
                // Stop heading override if we rotate too much on our own
                if (Math.abs(robotState.getRotation().getRadians() - headingOverride.getSetpoint()) > Units.degreesToRadians(headingOverrideThresholdDegrees.get())) {
                    headingOverride.setSetpoint(robotState.getRotation().getRadians());
                    headingOverrideSetpointResetTimer.stop();
                    shouldRunHeadingOverride = false;
                } else {
                    joystickSetpoint = new ChassisSpeeds(
                            joystickSetpoint.vxMetersPerSecond,
                            joystickSetpoint.vyMetersPerSecond,
                            headingOverride.calculate(robotState.getRotation().getRadians())
                    );
                }
            }
        } else {
            headingOverride.setSetpoint(robotState.getRotation().getRadians());
            headingOverrideSetpointResetTimer.stop();
            shouldRunHeadingOverride = false;
        }
        Logger.recordOutput("Drive/DriveJoystick/HeadingOverrideRunning", shouldRunHeadingOverride);

        return DriveRequest.chassisSpeeds(joystickSetpoint);
    }
}
