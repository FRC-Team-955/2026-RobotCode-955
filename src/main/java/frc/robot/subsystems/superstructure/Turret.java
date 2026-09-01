package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.devices.motor.CtrlSparkMaxConfig;
import frc.lib.devices.motor.MechanismSim;
import frc.lib.devices.motor.Motor;
import frc.lib.network.LoggedTunablePIDF;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Turret implements Periodic {
    // 0 = shooting away from intake
    private static final double minPositionRad = Units.degreesToRadians(-90);
    private static final double maxPositionRad = Units.degreesToRadians(90);
    private static final double initialPositionRad = 0.0;

    private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final Motor motor = Motor
            .createSparkMax(
                    "Superstructure/Turret",
                    11,
                    new CtrlSparkMaxConfig()
                            .withInverted(true)
                            .withCurrentLimit(60)
                            .withGearRatio(3.0 * 3.0 * (68.0 / 12.0))
                            .withNeutralMode(NeutralModeValue.Brake),
                    initialPositionRad,
                    MechanismSim.arm(
                            0.1004362678,
                            Units.inchesToMeters(7.5),
                            minPositionRad,
                            maxPositionRad,
                            false
                    )
            )
            .withPositionGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Turret/PositionGains")
                        .withP(0.1)
                        .withS(0.0, StaticFeedforwardSignValue.UseClosedLoopSign);
                case SIM -> new LoggedTunablePIDF("Superstructure/Turret/PositionGains")
                        .withP(1.0);
            })
            .withVelocityGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Turret/VelocityGains")
                        .withV(0.1);
                case SIM -> new LoggedTunablePIDF("Superstructure/Turret/VelocityGains")
                        .withV(1.0);
            });

    @RequiredArgsConstructor
    public enum Goal {
        SHOOT(() -> 0.0),
        AIM_AT_CLOSEST_HUB(() -> 0.0),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier setpointRad;
    }

    @Setter
    @Getter
    private Goal goal = Goal.AIM_AT_CLOSEST_HUB;

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State state = new TrapezoidProfile.State(initialPositionRad, 0.0);

    private final Debouncer emergencyStopDebouncer = new Debouncer(1.0, Debouncer.DebounceType.kRising);

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
        boolean shouldEmergencyStop = emergencyStopDebouncer.calculate(motor.getStatorCurrentAmps() >= 50);
        if (!motor.isEmergencyStopped()) {
            if ((shouldEmergencyStop || operatorDashboard.turretEStop.get()) && !BuildConstants.isSim) {
                motor.emergencyStop(NeutralModeValue.Coast);
                operatorDashboard.turretEStop.set(true);
            }
        } else {
            if (!operatorDashboard.turretEStop.get()) {
                motor.undoEmergencyStop(NeutralModeValue.Brake);
                operatorDashboard.turretEStop.set(false);
            }
        }

        // Apply network inputs
        if (!motor.isEmergencyStopped() && operatorDashboard.coastOverride.hasChanged()) {
            motor.setNeutralMode(operatorDashboard.coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Turret/Goal", goal);
        if (DriverStation.isDisabled() || motor.isEmergencyStopped()) {
            motor.setVoltageRequest(0.0);

            // Reset state to current position
            state = new TrapezoidProfile.State(motor.getPositionRad(), 0.0);
        } else {
            // See the comments above the lookaheadState and goalState variables for why we calculate two profiles

            double setpointRad = goal.setpointRad.getAsDouble();
            setpointRad = MathUtil.clamp(setpointRad, minPositionRad, maxPositionRad);
            if (BuildConstants.isSimOrReplay) {
                Logger.recordOutput("Superstructure/Turret/OriginalSetpointRad", setpointRad);
            }
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            state = profile.calculate(Constants.loopPeriod, state, wantedState);
            Logger.recordOutput("Superstructure/Turret/ProfileSetpointRad", state.position);

            motor.setMotionProfileRequest(state.position, state.velocity);
        }
    }

    public double getRobotRelativeHeadingRad() {
        // add 180° - see comment at top of class
        return motor.getPositionRad() + Math.PI;
    }

    public double getRobotRelativeHeadingVelocityRadPerSec() {
        return motor.getVelocityRadPerSec();
    }

    public Transform3d getMechanismTransform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(10.0), 0.0, Units.inchesToMeters(6.25)),
                new Rotation3d(0.0, Units.degreesToRadians(90.0), 0.0)).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, -motor.getPositionRad(), 0.0)
        ));
    }
}
