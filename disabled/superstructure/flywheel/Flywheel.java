package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.EnergyLogger;
import frc.lib.Util;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.OperatorDashboard;
import frc.robot.shooting.ShootingKinematics;
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
    private static final EnergyLogger energyLogger = EnergyLogger.get();


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
    public final Alert highTemperatureAlert = new Alert("Flywheel motor temperature is high.", Alert.AlertType.kWarning);

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
        highTemperatureAlert.set(Math.max(inputs.leader.temperatureCelsius, inputs.follower.temperatureCelsius) > 50);

        energyLogger.reportPowerUsage("Flywheel",
                inputs.leader.connected ? inputs.leader.appliedVolts * inputs.leader.supplyCurrentAmps : 0.0,
                inputs.follower.connected ? inputs.follower.appliedVolts * inputs.follower.supplyCurrentAmps : 0.0);

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
            io.setVelocityRequest(value);
            if (BuildConstants.isSimOrReplay) Logger.recordOutput("Superstructure/Flywheel/RequestValue", value);
        }
    }


    public double getPositionRad() {
        return inputs.leader.positionRad;
    }

    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.leader.velocityRadPerSec);
    }

    public boolean isDisconnected() {
        return !inputs.leader.connected || !inputs.follower.connected;
    }

    public double getSetpointRPM() {
        return goal.setpointRPM.getAsDouble();
    }

    public Transform3d transform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(-6.910046), Units.inchesToMeters(-9.109744), Units.inchesToMeters(12.861381)),
                new Rotation3d(0.0, 0.0, 0.0)
        ).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, getPositionRad(), 0.0)
        ));
    }
}
