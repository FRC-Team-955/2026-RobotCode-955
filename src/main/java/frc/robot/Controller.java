package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Util;
import frc.lib.controller.CommandSteamInputController;
import frc.lib.subsystem.Periodic;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class Controller implements Periodic {
    public enum Mode {

    }

    private final CommandXboxController controller;
    private final Alert controllerDisconnectedAlert = new Alert("Driver controller is not connected!", Alert.AlertType.kError);

    // Intermediates - used in assist calculations
    private Rotation2d linearDirection = new Rotation2d();
    private double linearMagnitude = 0.0;

    // Results - used for drive/assist calculations
    @Getter
    private ChassisSpeeds setpointFieldRelative = new ChassisSpeeds();

    private static Controller instance;

    public static Controller get() {
        if (instance == null)
            synchronized (Controller.class) {
                instance = new Controller();
            }

        return instance;
    }

    private Controller() {
        if (BuildConstants.mode == BuildConstants.Mode.SIM && System.getProperty("os.name").contains("Mac OS X")) {
            controller = new CommandSteamInputController(0);
//            controller = new CommandNintendoSwitchProController(0);
        } else {
            controller = new CommandXboxController(0);
        }
    }

    @Override
    public void periodicBeforeCommands() {
        controllerDisconnectedAlert.set(!controller.isConnected());

        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        // forward on joystick is negative y - we want positive x for forward
        double x = -controller.getLeftY();
        // right on joystick is positive x - we want negative y for right
        double y = -controller.getLeftX();
        // right on joystick is positive x - we want negative x for right (CCW is positive)
        double omega = -controller.getRightX();

        Logger.recordOutput("JoystickDrive/Suppliers/X", x);
        Logger.recordOutput("JoystickDrive/Suppliers/Y", y);
        Logger.recordOutput("JoystickDrive/Suppliers/Omega", omega);

        linearMagnitude = MathUtil.clamp(MathUtil.applyDeadband(Math.hypot(x, y), joystickDriveDeadband), -1, 1);
        linearMagnitude = linearMagnitude * linearMagnitude;

        double omegaMagnitude = MathUtil.applyDeadband(omega, joystickDriveDeadband);
        omegaMagnitude = Math.copySign(omegaMagnitude * omegaMagnitude, omegaMagnitude);

        // Scale linear magnitude by omega - when going full omega, want half linear
        linearMagnitude *= MathUtil.clamp(1.0 - Math.abs(omegaMagnitude / 2.0), 0.5, 1.0);

        // If x and y are both 0, Rotation2d will not be happy
        // Intermediates - used in assist calculations
        Translation2d linearVelocity;
        if (x != 0 || y != 0) {
            linearDirection = new Rotation2d(x, y);
            linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                    .getTranslation();
        } else {
            // Linear magnitude should be 0 anyways
            linearDirection = new Rotation2d();
            linearVelocity = new Translation2d();
        }

        Logger.recordOutput("JoystickDrive/LinearMagnitude", linearMagnitude);
        Logger.recordOutput("JoystickDrive/LinearDirection", linearDirection);
        Logger.recordOutput("JoystickDrive/LinearVelocity", linearVelocity);
        Logger.recordOutput("JoystickDrive/OmegaMagnitude", omegaMagnitude);

        if (Util.shouldFlip()) {
            linearVelocity = linearVelocity.rotateBy(Rotation2d.k180deg);
        }
        setpointFieldRelative = new ChassisSpeeds(
                linearVelocity.getX() * driveConfig.moduleLimits().maxDriveVelocityMetersPerSec(),
                linearVelocity.getY() * driveConfig.moduleLimits().maxDriveVelocityMetersPerSec(),
                omegaMagnitude * joystickMaxAngularSpeedRadPerSec
        );
    }

    public boolean shouldAssist(Pose2d currentPose, Pose2d assistPose) {
        Logger.recordOutput("JoystickDrive/Assist/Pose", assistPose);

        // Get the translation between robot and assist
        Translation2d robotToAssist = assistPose.getTranslation().minus(currentPose.getTranslation());
        // Calculate direction from robot to assist
        Rotation2d robotToAssistDirection = robotToAssist.getAngle();
        Logger.recordOutput("JoystickDrive/Assist/RobotToAssistDirection", robotToAssistDirection);

        // Flip joystick direction to match robot to assist direction
        // Joystick direction is relative to alliance wall and needs to be flipped on red alliance to match origin
        Rotation2d joystickLinearDirectionFlipped = Util.flipIfNeeded(linearDirection);
        Logger.recordOutput("JoystickDrive/Assist/FlippedJoystickLinearDirection", joystickLinearDirectionFlipped);

        // Get difference between joystick direction and assist direction
        Rotation2d directionDiff = robotToAssistDirection.minus(joystickLinearDirectionFlipped);
        Logger.recordOutput("JoystickDrive/Assist/DirectionDifference", directionDiff);

        // Get distance to assist pose
        double distanceToAssist = currentPose.getTranslation().getDistance(assistPose.getTranslation());
        Logger.recordOutput("JoystickDrive/Assist/DistanceToAssist", distanceToAssist);

        // If we are:
        if (
            // - moving linearly in some way (if we are only rotating, don't assist)
                linearMagnitude != 0.0 &&
                        // - going towards the assist pose based on threshold
                        Math.abs(directionDiff.getRadians()) < assistDirectionToleranceRad &&
                        // - close enough to assist pose
                        distanceToAssist < assistMaximumDistanceMeters
        ) {
            // then use automatic control.
            Logger.recordOutput("JoystickDrive/Assist/Running", true);
            return true;
        } else {
            Logger.recordOutput("JoystickDrive/Assist/Running", false);
            return false;
        }
    }

    public ChassisSpeeds getDriveSetpointRobotRelative(Rotation2d robotAngle) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(setpointFieldRelative, robotAngle);
    }

    public Command rumble(double value, double timeSeconds) {
        return Commands.startEnd(
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, value),
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0)
        ).withTimeout(timeSeconds);
    }

    public Trigger a() {
        return controller.a();
    }

    public Trigger b() {
        return controller.b();
    }

    public Trigger x() {
        return controller.x();
    }

    public Trigger y() {
        return controller.y();
    }

    public Trigger leftBumper() {
        return controller.leftBumper();
    }

    public Trigger rightBumper() {
        return controller.rightBumper();
    }

    public Trigger back() {
        return controller.back();
    }

    public Trigger start() {
        return controller.start();
    }

    public Trigger leftStick() {
        return controller.leftStick();
    }

    public Trigger rightStick() {
        return controller.rightStick();
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than 0.5.
     */
    public Trigger leftTrigger() {
        return controller.leftTrigger();
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than 0.5.
     */
    public Trigger rightTrigger() {
        return controller.rightTrigger();
    }

    /**
     * Get the X axis value of left side of the controller. Right is positive.
     */
    public double getLeftX() {
        return controller.getLeftX();
    }

    /**
     * Get the X axis value of right side of the controller. Right is positive.
     */
    public double getRightX() {
        return controller.getRightX();
    }

    /**
     * Get the Y axis value of left side of the controller. Back is positive.
     */
    public double getLeftY() {
        return controller.getLeftY();
    }

    /**
     * Get the Y axis value of right side of the controller. Back is positive.
     */
    public double getRightY() {
        return controller.getRightY();
    }

    /**
     * Get the left trigger axis value of the controller. Note that this axis is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     */
    public double getLeftTriggerAxis() {
        return controller.getLeftTriggerAxis();
    }

    /**
     * Get the right trigger axis value of the controller. Note that this axis is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     */
    public double getRightTriggerAxis() {
        return controller.getRightTriggerAxis();
    }
}
