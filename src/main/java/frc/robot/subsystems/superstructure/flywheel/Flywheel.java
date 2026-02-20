package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import frc.robot.ShootingKinematics;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.*;

public class Flywheel implements Periodic {
    private static final LoggedTunableNumber shootHubManualMetersPerSec = new LoggedTunableNumber("Superstructure/Flywheel/Goal/ShootHubManualMetersPerSec", 5.0);
    private static final LoggedTunableNumber shootTowerManualMetersPerSec = new LoggedTunableNumber("Superstructure/Flywheel/Goal/ShootTowerManualMetersPerSec", 5.0);
    private static final LoggedTunableNumber passManualMetersPerSec = new LoggedTunableNumber("Superstructure/Flywheel/Goal/PassManualMetersPerSec", 5.0);
    private static final LoggedTunableNumber ejectRPM = new LoggedTunableNumber("Superstructure/Flywheel/Goal/EjectRPM", -300);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private final FlywheelIO io = createIO();
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0),
        SHOOT_AND_PASS_AUTOMATIC(() -> shootingKinematics.getShootingParameters().velocityMetersPerSec() / flywheelRadiusMeters),
        SHOOT_HUB_MANUAL(() -> shootHubManualMetersPerSec.get() / flywheelRadiusMeters),
        SHOOT_TOWER_MANUAL(() -> shootTowerManualMetersPerSec.get() / flywheelRadiusMeters),
        PASS_MANUAL(() -> passManualMetersPerSec.get() / flywheelRadiusMeters),
        EJECT(() -> Units.rotationsPerMinuteToRadiansPerSecond(ejectRPM.get())),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

    private final Alert leaderMotorDisconnectedAlert = new Alert("Flywheel leader motor is disconnected.", Alert.AlertType.kError);
    private final Alert followerMotorDisconnectedAlert = new Alert("Flywheel follower motor is disconnected.", Alert.AlertType.kError);

    private static Flywheel instance;

    public static synchronized Flywheel get() {
        if (instance == null) {
            instance = new Flywheel();
        }

        return instance;
    }

    private Flywheel() {
        if (instance != null) {
            Util.error("Duplicate Flywheel created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Flywheel", inputs);

        leaderMotorDisconnectedAlert.set(!inputs.leader.connected);
        followerMotorDisconnectedAlert.set(!inputs.follower.connected);

        if (velocityGains.hasChanged()) {
            io.setVelocityPIDF(velocityGains);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Flywheel/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setStopRequest();
        } else {
            double value = goal.value.getAsDouble();
            Logger.recordOutput("Superstructure/Flywheel/RequestValue", value);
            io.setVelocityRequest(value);
        }
    }

    public double getPositionRad() {
        return inputs.leader.positionRad;
    }

    public double getVelocityMetersPerSec() {
        return inputs.leader.velocityRadPerSec * FlywheelConstants.flywheelRadiusMeters;
    }

    @AutoLogOutput(key = "Superstructure/Flywheel/AtGoal")
    public boolean atGoal() {
        double value = goal.value.getAsDouble();
        return Math.abs(inputs.leader.velocityRadPerSec - value) <= velocityToleranceRadPerSec;
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }
}
