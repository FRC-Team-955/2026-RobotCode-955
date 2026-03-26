// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.EnergyLogger;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.OperatorDashboard;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class Module {
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final EnergyLogger energyLogger = EnergyLogger.get();

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private PIDController turnAbsolutePID = moduleConfig.turnAbsoluteGains().toPIDWrapRadians();
    private final Debouncer turnAbsoluteRelativeDifferenceDebouncer = new Debouncer(1.0, Debouncer.DebounceType.kRising);

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    private final Alert turnEncoderDisparityAlert;
    private final Alert turnEncoderDisparityStickyAlert;

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        driveDisconnectedAlert = new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
        turnDisconnectedAlert = new Alert("Disconnected turn motor on module " + index + ".", AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert("Disconnected turn encoder on module " + index + ".", AlertType.kError);
        turnEncoderDisparityAlert = new Alert("Absolute and relative turn encoders on module " + index + " are not matching up.", AlertType.kError);
        turnEncoderDisparityStickyAlert = new Alert("Absolute and relative turn encoders on module " + index + " didn't match up, but they do now.", AlertType.kWarning);
    }

    public void updateAndProcessInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Drive/Module" + index, inputs);
    }

    public void periodicBeforeCommands() {
        energyLogger.reportCurrentUsage(
                "Module/Drive/" + index,
                inputs.driveConnected ? inputs.driveSupplyCurrentAmps : 0.0
        );
        energyLogger.reportCurrentUsage(
                "Module/Turn/" + index,
                inputs.turnConnected ? inputs.turnSupplyCurrentAmps : 0.0
        );

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnAbsoluteEncoderConnected);
        boolean turnEncoderDisparity = turnAbsoluteRelativeDifferenceDebouncer.calculate(
                Math.abs(
                        MathUtil.angleModulus(inputs.turnAbsolutePositionRad)
                                - MathUtil.angleModulus(inputs.turnPositionRad)
                ) > Units.degreesToRadians(3.0)
        );
        turnEncoderDisparityAlert.set(turnEncoderDisparity);

        // Zero turn if there is a disparity and we aren't moving
        if (turnEncoderDisparity &&
                Math.abs(getDriveVelocityMetersPerSec()) < 1e-4 &&
                inputs.turnAbsoluteEncoderConnected &&
                inputs.turnAbsoluteEncoderMagnetHealth != MagnetHealthValue.Magnet_Invalid
        ) {
            io.setTurnRelativeEncoderFromAbsolute();
            turnEncoderDisparityStickyAlert.set(true);
        }
    }

    public void runSetpoint(SwerveModuleState state) {
        if (disableDriving) {
            io.setDriveOpenLoop(0.0);
        } else {
            io.setDriveClosedLoop(state.speedMetersPerSecond / driveConfig.wheelRadiusMeters());
        }

        // Anti-jitter
        if (Math.abs(state.speedMetersPerSecond) < 1e-4 && Math.abs(getTurnAngle().minus(state.angle).getRadians()) < 0.1) {
            io.setTurnOpenLoop(0.0);
        } else {
            setTurnClosedLoop(state.angle.getRadians());
        }
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        setTurnClosedLoop(0.0);
    }

    private void setTurnClosedLoop(double positionRad) {
        if (operatorDashboard.driveTurnAbsolutePID.get()) {
            io.setTurnOpenLoop(turnAbsolutePID.calculate(inputs.turnAbsolutePositionRad, positionRad));
        } else {
            io.setTurnClosedLoop(positionRad);
        }
    }

    /**
     * Disables all outputs to motors.
     */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    public void setDrivePIDF(LoggedTunablePIDF newGains) {
        io.setDrivePIDF(newGains);
    }

    public void setTurnRelativePIDF(LoggedTunablePIDF newGains) {
        io.setTurnRelativePIDF(newGains);
    }

    public void setTurnAbsolutePIDF(LoggedTunablePIDF newGains) {
        turnAbsolutePID = newGains.toPIDWrapRadians();
    }

    public void setBrakeMode(boolean enable) {
        io.setDriveBrakeMode(enable);
        io.setTurnBrakeMode(enable);
    }

    /**
     * Returns the current turn angle of the module.
     */
    public Rotation2d getTurnAngle() {
        return new Rotation2d(MathUtil.angleModulus(
                operatorDashboard.driveTurnAbsolutePID.get()
                        ? inputs.turnAbsolutePositionRad
                        : inputs.turnPositionRad
        ));
    }

    public double getDrivePositionRad() {
        return inputs.drivePositionRad;
    }

    public double getDriveVelocityRadPerSec() {
        return inputs.driveVelocityRadPerSec;
    }

    /**
     * Returns the current drive position of the module in meters.
     */
    public double getDrivePositionMeters() {
        return inputs.drivePositionRad * driveConfig.wheelRadiusMeters();
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getDriveVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * driveConfig.wheelRadiusMeters();
    }

    public double getDriveStatorCurrentAmps() {
        return inputs.driveStatorCurrentAmps;
    }

    /**
     * Returns the module position (turn angle and drive position).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getTurnAngle());
    }

    /**
     * Returns the module state (turn angle and drive velocity).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSec(), getTurnAngle());
    }

    /**
     * Returns the timestamps of the samples received this cycle.
     */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public double[] getOdometryDrivePositionsRad() {
        return inputs.odometryDrivePositionsRad;
    }

    public double[] getOdometryTurnPositionsRad() {
        return inputs.odometryTurnPositionsRad;
    }
}
