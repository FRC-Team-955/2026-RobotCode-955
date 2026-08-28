package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.EnergyLogger;
import frc.lib.Util;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.superstructure.turret.TurretConstants.*;

public class Turret implements Periodic {
    private static final LoggedTunableNumber profileLookaheadTimeSec = new LoggedTunableNumber("Superstructure/Turret/ProfileLookaheadTimeSec", 0.15);
    private static final LoggedTunableNumber stowSetpointDegrees = new LoggedTunableNumber("Superstructure/Turret/Goal/StowDegrees", 70.0);

    private static final EnergyLogger energyLogger = EnergyLogger.get();

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final RobotState robotState = RobotState.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();


    @RequiredArgsConstructor
    public enum Goal {
        SHOOT(() -> Units.degreesToRadians(stowSetpointDegrees.get())),
        AIM_AT_CLOSEST_HUB(() -> minPositionRad),
        ;

        private final DoubleSupplier setpointRad;
    }

    @Setter
    @Getter
    private Goal goal = Goal.AIM_AT_CLOSEST_HUB;

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private Double lastSetpointRad = null;
    // goalState is just for logging the profile we want to follow.
    // lookaheadState is shifted some seconds into the future, and is used for PID setpoint.
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private TrapezoidProfile.State lookaheadState = new TrapezoidProfile.State();

    @Getter
    private boolean emergencyStopped = false;

    private final Alert motorDisconnectedAlert = new Alert("Turret motor is disconnected.", Alert.AlertType.kError);
    public final Alert highTemperatureAlert = new Alert("Turret motor temperature is high.", Alert.AlertType.kWarning);
    private final Alert emergencyStoppedAlert = new Alert("Turret is E-stopped!", Alert.AlertType.kError);

    private static Turret instance;

    public static synchronized Turret get() {
        if (instance == null) {
            instance = new Turret();
        }

        return instance;
    }

    private Turret() {
        if (instance != null) {
            Util.error("Duplicate Turret created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Turret", inputs);

        motorDisconnectedAlert.set(!inputs.connected);
        highTemperatureAlert.set(inputs.temperatureCelsius > 50);

        energyLogger.reportPowerUsage("Turret", inputs.connected ? inputs.appliedVolts * inputs.supplyCurrentAmps : 0.0);

        if (!emergencyStopped) {
            if (operatorDashboard.turretEStop.get()) {
                io.setRequest(RequestType.VoltageVolts, 0.0);
                emergencyStopped = true;
                operatorDashboard.turretEStop.set(true);
            }
        } else {
            if (!operatorDashboard.turretEStop.get()) {
                // Let operator turn off e-stop
                emergencyStopped = false;
                operatorDashboard.turretEStop.set(false);
            }
        }
        emergencyStoppedAlert.set(emergencyStopped);

        // Apply network inputs
        if (gains.hasChanged()) {
            io.setPositionPIDF(gains);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Turret/Goal", goal);

        if (DriverStation.isDisabled() || emergencyStopped) {
            io.setRequest(RequestType.VoltageVolts, 0.0);

            // Reset states to current position
            goalState = new TrapezoidProfile.State(inputs.positionRad, 0.0);
            lookaheadState = goalState;
        } else {
            double setpointRad = goal.setpointRad.getAsDouble();
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            if (BuildConstants.isSimOrReplay)
                Logger.recordOutput("Superstructure/Turret/OriginalSetpointRad", setpointRad);
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            if (lastSetpointRad == null || setpointRad != lastSetpointRad) {
                // Setpoint changed - shift setpoint profile into the future
                lookaheadState = profile.calculate(profileLookaheadTimeSec.get(), lookaheadState, wantedState);
            }
            lastSetpointRad = setpointRad;

            goalState = profile.calculate(Constants.loopPeriod, goalState, wantedState);
            if (BuildConstants.isSimOrReplay)
                Logger.recordOutput("Superstructure/Turret/ProfileSetpointRad", goalState.position);

            lookaheadState = profile.calculate(Constants.loopPeriod, lookaheadState, wantedState);
            if (BuildConstants.isSimOrReplay)
                Logger.recordOutput("Superstructure/Turret/LookaheadSetpointRad", lookaheadState.position);

            io.setRequest(RequestType.PositionRad, lookaheadState.position);
        }
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    public boolean isDisconnected() {
        return !inputs.connected;
    }

    public Transform3d transform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(10.0), 0.0, Units.inchesToMeters(6.25)),
                new Rotation3d(0.0, Units.degreesToRadians(90.0), 0.0)).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, -getPositionRad(), 0.0)
        ));
    }
}
