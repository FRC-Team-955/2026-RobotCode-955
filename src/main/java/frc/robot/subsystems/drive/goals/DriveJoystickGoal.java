package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.lib.SlewRateLimiter2d;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.controller.Controller;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.*;


@RequiredArgsConstructor
public class DriveJoystickGoal extends DriveGoal {
    public enum Mode {
        Normal,
        StopWithX,
        Aim,
        Assist,
        AimAndAssist,
    }

    private static final LoggedTunableNumber headingOverrideSetpointResetTime = new LoggedTunableNumber("Drive/DriveJoystick/HeadingOverrideSetpointResetTimeSeconds", 0.25);
    private static final LoggedTunableNumber headingOverrideThresholdDegrees = new LoggedTunableNumber("Drive/DriveJoystick/HeadingOverrideThresholdDegrees", 30.0);

    private static final LoggedTunableNumber aimMaxLinearVelocityMetersPerSec = new LoggedTunableNumber("Drive/DriveJoystick/Aim/MaxLinearVelocity", 2);
    private static final LoggedTunableNumber aimMaxLinearAccelerationMetersPerSecPerSec = new LoggedTunableNumber("Drive/DriveJoystick/Aim/MaxLinearAcceleration", 5);

    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private static final GamePieceVision gamePieceVision = GamePieceVision.get();

    private static final Supplier<Optional<Translation2d>> assistTranslationSupplier = () -> gamePieceVision.getBestTargets().stream().findFirst();

    private final Mode mode;

    private final PIDController headingOverride = headingOverrideGains
            .toPIDWrapRadians(
                    moveToConfig.angularPositionToleranceRad().get(),
                    moveToConfig.angularVelocityToleranceRadPerSec().get()
            );
    private final Debouncer headingOverrideEnabledDebouncer = new Debouncer(headingOverrideSetpointResetTime.get(), Debouncer.DebounceType.kRising);
    private boolean runningHeadingOverride = false;

    private final SlewRateLimiter2d aimLinearAccelerationLimiter = new SlewRateLimiter2d(aimMaxLinearAccelerationMetersPerSecPerSec.get(), robotState.getMeasuredChassisSpeeds());

    private final PIDController assistPID = assistGains.toPID();

    private Rotation2d lastLinearDirection = new Rotation2d();

    @Override
    public DriveRequest getRequest() {
        if (headingOverrideGains.hasChanged()) {
            headingOverrideGains.applyPID(headingOverride);
        }
        if (headingOverrideSetpointResetTime.hasChanged()) {
            headingOverrideEnabledDebouncer.setDebounceTime(headingOverrideSetpointResetTime.get());
        }

        if (aimMaxLinearAccelerationMetersPerSecPerSec.hasChanged()) {
            aimLinearAccelerationLimiter.setLimit(aimMaxLinearAccelerationMetersPerSecPerSec.get());
        }

        if (assistGains.hasChanged()) {
            assistGains.applyPID(assistPID);
        }

        //////////////////////////////////////////////////////////////////////

        Rotation2d linearDirection = controller.getDriveLinearDirection();
        double linearMagnitude = controller.getDriveLinearMagnitude();
        // Keep going in last direction if command is 0
        if (linearMagnitude == 0.0) {
            linearDirection = lastLinearDirection;
        } else {
            lastLinearDirection = linearDirection;
        }

        if (mode == Mode.Assist || mode == Mode.AimAndAssist) {
            // Adjust linear direction in Y axis while assisting
            Logger.recordOutput("Drive/AssistRunning", false);
            var optionalAssistTranslation = assistTranslationSupplier.get();
            if (optionalAssistTranslation.isPresent()) {
                Translation2d assistTranslation = optionalAssistTranslation.get();

                if (controller.shouldAssist(robotState.getPose(), assistTranslation)) {
                    Logger.recordOutput("Drive/AssistRunning", true);

                    // First convert to robot relative
                    Rotation2d linearDirectionRobotRelative = linearDirection.rotateBy(robotState.getRotation().unaryMinus());

                    double yDist = new Transform2d(
                            robotState.getPose(),
                            new Pose2d(assistTranslation, robotState.getRotation())
                    ).getY();

                    // Adjust Y (left/right) to go towards game piece
                    // Note that there is a negative sign
                    linearDirection = new Rotation2d(linearDirectionRobotRelative.getCos(), -assistPID.calculate(yDist, 0))
                            // Rotate back to field relative
                            .rotateBy(robotState.getRotation());
                }
            }
        }

        Translation2d linearSetpoint;
        if (mode == Mode.Aim || mode == Mode.AimAndAssist) {
            // Limit linear velocity and acceleration while aiming
            linearSetpoint = new Translation2d(
                    linearMagnitude * aimMaxLinearVelocityMetersPerSec.get(),
                    linearDirection
            );
            linearSetpoint = aimLinearAccelerationLimiter.calculate(linearSetpoint);
        } else {
            linearSetpoint = new Translation2d(
                    linearMagnitude * driveConfig.maxVelocityMetersPerSec(),
                    linearDirection
            );
            // Reset with latest speeds
            aimLinearAccelerationLimiter.reset(robotState.getMeasuredChassisSpeeds());
        }

        if (mode == Mode.Aim || mode == Mode.AimAndAssist) {
            // Set heading override setpoint for aiming
            headingOverride.setSetpoint(shootingKinematics.getShootingParameters().headingRad());
        }

        double angularSetpoint;
        if (
                headingOverrideEnabledDebouncer.calculate(
                        (
                                // Stop heading override if we are running and rotate too much on our own
                                !runningHeadingOverride ||
                                        Math.abs(robotState.getRotation().getRadians() - headingOverride.getSetpoint())
                                                > Units.degreesToRadians(headingOverrideThresholdDegrees.get())
                        ) && controller.getDriveAngularMagnitude() == 0.0
                ) ||
                        mode == Mode.Aim ||
                        mode == Mode.AimAndAssist
        ) {
            if (!runningHeadingOverride) {
                // Set PID setpoint
                headingOverride.reset();
                if (mode != Mode.Aim && mode != Mode.AimAndAssist) {
                    // Aiming setpoint is set above
                    headingOverride.setSetpoint(robotState.getRotation().getRadians());
                }
                runningHeadingOverride = true;
            }

            angularSetpoint = headingOverride.calculate(robotState.getRotation().getRadians());
            if (mode == Mode.Aim || mode == Mode.AimAndAssist) {
                angularSetpoint += shootingKinematics.rotationAboutHubRadiansPerSec(linearSetpoint);
            } else {
                // limit to drive linear magnitude when not aiming
                // drive linear magnitude is between 0 and 1
                angularSetpoint *= linearMagnitude;
            }

            boolean headingOverrideAtSetpoint = headingOverride.atSetpoint();
            //Logger.recordOutput("Drive/DriveJoystick/HeadingOverrideAtSetpoint", headingOverrideAtSetpoint);
            if (headingOverrideAtSetpoint) {
                angularSetpoint = 0.0;
            }
        } else {
            runningHeadingOverride = false;

            angularSetpoint = controller.getDriveAngularMagnitude() * joystickMaxAngularSpeedRadPerSec;
        }
        //Logger.recordOutput("Drive/DriveJoystick/HeadingOverrideRunning", runningHeadingOverride);

        if (
                (mode == Mode.Aim || mode == Mode.AimAndAssist || mode == Mode.StopWithX) &&
                        linearSetpoint.getNorm() == 0.0 &&
                        angularSetpoint == 0.0
        ) {
            return DriveRequest.stopWithX();
        } else {
            return DriveRequest.chassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearSetpoint.getX(),
                    linearSetpoint.getY(),
                    angularSetpoint,
                    robotState.getRotation()
            ));
        }
    }
}
