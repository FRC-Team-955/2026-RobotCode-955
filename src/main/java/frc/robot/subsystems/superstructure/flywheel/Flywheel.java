package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import frc.robot.ShootingKinematics;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.createIO;
import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.velocityGains;

public class Flywheel implements Periodic {
    private static final LoggedTunableNumber ejectRPM = new LoggedTunableNumber("Superstructure/Flywheel/Goal/EjectRPM", -300);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private final FlywheelIO io = createIO();
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0),
        SHOOT(() -> shootingKinematics.getShootingParameters().velocityRPM()),
        EJECT(ejectRPM::get),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier setpointRPM;
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
        if (DriverStation.isDisabled() || goal == Goal.IDLE) {
            io.setStopRequest();
        } else {
            double value = Units.rotationsPerMinuteToRadiansPerSecond(goal.setpointRPM.getAsDouble());
            Logger.recordOutput("Superstructure/Flywheel/RequestValue", value);
            io.setVelocityRequest(value);
        }
    }


    public double getPositionRad() {
        return inputs.leader.positionRad;
    }

    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.leader.velocityRadPerSec);
    }

    public double getSetpointRPM() {
        return goal.setpointRPM.getAsDouble();
    }
}
