package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.Controller;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.headingOverrideGains;


@RequiredArgsConstructor
public class DriveJoystickGoal extends DriveGoal {
    private static final LoggedTunableNumber headingOverrideSetpointResetTime = new LoggedTunableNumber("Drive/DriveJoystick/HeadingOverrideSetpointResetTimeSeconds", 0.25);
    private static final LoggedTunableNumber headingOverrideThresholdDegrees = new LoggedTunableNumber("Drive/DriveJoystick/HeadingOverrideThresholdDegrees", 30.0);

    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();

    private final PIDController headingOverride = headingOverrideGains.toPIDWrapRadians();
    private final Debouncer headingOverrideEnabledDebouncer = new Debouncer(headingOverrideSetpointResetTime.get(), Debouncer.DebounceType.kRising);
    private boolean runningHeadingOverride = false;

    @Override
    public DriveRequest getRequest() {
        if (headingOverrideGains.hasChanged()) {
            headingOverrideGains.applyPID(headingOverride);
        }
        if (headingOverrideSetpointResetTime.hasChanged()) {
            headingOverrideEnabledDebouncer.setDebounceTime(headingOverrideSetpointResetTime.get());
        }

        //////////////////////////////////////////////////////////////////////

        ChassisSpeeds joystickSetpoint = controller.getDriveSetpointRobotRelative(robotState.getRotation());

        boolean canRunHeadingOverride = headingOverrideEnabledDebouncer.calculate(
                (
                        // Stop heading override if we are running and rotate too much on our own
                        !runningHeadingOverride ||
                                Math.abs(robotState.getRotation().getRadians() - headingOverride.getSetpoint())
                                        > Units.degreesToRadians(headingOverrideThresholdDegrees.get())
                ) && joystickSetpoint.omegaRadiansPerSecond == 0.0
        );
        if (canRunHeadingOverride) {
            if (!runningHeadingOverride) {
                // Set PID setpoint
                headingOverride.reset();
                headingOverride.setSetpoint(robotState.getRotation().getRadians());
                runningHeadingOverride = true;
            }

            joystickSetpoint = new ChassisSpeeds(
                    joystickSetpoint.vxMetersPerSecond,
                    joystickSetpoint.vyMetersPerSecond,
                    // limit to drive linear magnitude
                    // drive linear magnitude is between 0 and 1
                    headingOverride.calculate(robotState.getRotation().getRadians()) * controller.getDriveLinearMagnitude()
            );
        } else {
            runningHeadingOverride = false;
        }
        Logger.recordOutput("Drive/DriveJoystick/HeadingOverrideRunning", runningHeadingOverride);

        return DriveRequest.chassisSpeeds(joystickSetpoint);
    }
}
