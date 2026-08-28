package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.shooting.ShootingKinematics;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.superstructure.turret.TurretConstants.*;

public class Turret implements Periodic {
    private static final RobotState robotState = RobotState.get();
    private static final LoggedTunableNumber profileLookaheadTimeSec = new LoggedTunableNumber("Superstructure/Turret/ProfileLookaheadTimeSec", 0.15);

    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        AIM(() -> MathUtil.angleModulus(
                ShootingKinematics.get().getShootingParameters().headingRad()
                        - robotState.getPose().getRotation().getRadians()));

        private final DoubleSupplier setpointRad;
    }

    @Setter
    @Getter
    private Turret.Goal goal = Turret.Goal.AIM;

    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private Double lastSetpointRad = null;
    // goalState is just for logging the profile we want to follow.
    // lookaheadState is shifted some seconds into the future, and is used for PID setpoint.
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private TrapezoidProfile.State lookaheadState = new TrapezoidProfile.State();

    private Double homingAngleAtMaxRad = null;
    @Getter
    private boolean homed = false;
    // Measured at homing, so the range comes from the mechanism instead of a hardcoded guess.
    // Zero sits midway between the stops by construction.
    @Getter
    private double minPositionRad = 0.0;
    @Getter
    private double maxPositionRad = 0.0;

    private final Alert motorDisconnectedAlert = new Alert("Turret motor is disconnected.", Alert.AlertType.kError);
    private final Alert homingTravelAlert = new Alert(
            "Turret homing points were too close together - hold it against each stop and retry.",
            Alert.AlertType.kError
    );
    private final Alert outOfRangeAlert = new Alert(
            "Turret can't reach the shot - the drivebase needs to turn.",
            Alert.AlertType.kWarning
    );

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

        // Nobody can push the turret into a hard stop in sim, so there is nothing to home against
        if (BuildConstants.isSim) {
            setTravelRad(simTravelRad);
            homed = true;
            operatorDashboard.turretNotHomedAlert.set(false);
        }
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Turret", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setNeutralMode(operatorDashboard.coastOverride.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        }

        if (gains.hasChanged()) {
            io.setPositionPIDF(gains);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Turret/Goal", goal);
        Logger.recordOutput("Superstructure/Turret/Homed", homed);
        Logger.recordOutput("Superstructure/Turret/TravelDeg", Units.radiansToDegrees(maxPositionRad - minPositionRad));
        // Before homing the encoder is off by an unknown amount, so a position request would
        // drive the turret somewhere arbitrary - possibly straight into a hard stop.
        if (DriverStation.isDisabled() || !homed) {
            io.setRequest(RequestType.VoltageVolts, 0);
            outOfRangeAlert.set(false);

            // Reset states to current position
            goalState = new TrapezoidProfile.State(inputs.positionRad, 0.0);
            lookaheadState = goalState;
        } else {
            // See the comments above the lookaheadState and goalState variables for why we effectively calculate two profiles

            double wantedRad = goal.setpointRad.getAsDouble();
            double setpointRad = MathUtil.clamp(wantedRad, minPositionRad, maxPositionRad);
            // Travel short of a full turn leaves a blind arc the turret simply can't cover
            outOfRangeAlert.set(setpointRad != wantedRad);
            //            Logger.recordOutput("Turret/OriginalSetpointRad", setpointRad);
            TrapezoidProfile.State wantedState = new TrapezoidProfile.State(setpointRad, 0.0);

            if (lastSetpointRad == null || setpointRad != lastSetpointRad) {
                // Setpoint changed - shift setpoint profile into the future
                lookaheadState = profile.calculate(profileLookaheadTimeSec.get(), lookaheadState, wantedState);
            }
            lastSetpointRad = setpointRad;

            goalState = profile.calculate(Constants.loopPeriod, goalState, wantedState);
            Logger.recordOutput("Superstructure/Turret/ProfileSetpointRad", goalState.position);

            lookaheadState = profile.calculate(Constants.loopPeriod, lookaheadState, wantedState);
            Logger.recordOutput("Superstructure/Turret/LookaheadSetpointRad", lookaheadState.position);

            io.setRequest(RequestType.PositionRad, lookaheadState.position);
        }
    }

    /**
     * Intended to be plugged into component rotation in RobotMechanism
     */
    public double getPositionRad() {
        return inputs.positionRad;
    }

    /**
     * Field-relative direction the turret is pointing: chassis heading plus turret angle.
     */
    @AutoLogOutput(key = "Superstructure/Turret/HeadingRad")
    public double getHeadingRad() {
        return MathUtil.angleModulus(robotState.getPose().getRotation().getRadians() + inputs.positionRad);
    }

    @AutoLogOutput(key = "Superstructure/Turret/AtGoal")
    public boolean atGoal() {
        if (!homed) {
            return false;
        }

        double value = goal.setpointRad.getAsDouble();
        return Math.abs(inputs.positionRad - value) <= positionToleranceRad;
    }

    /**
     * Records one end of the turret's travel. Call it with the turret held against the CCW (max)
     * stop, then again against the CW (min) stop, and the second call finishes homing.
     * <p>
     * There is no absolute encoder, so the two hard stops are the only fixed reference available.
     */
    public void captureHomingPoint() {
        if (homingAngleAtMaxRad == null) {
            homingAngleAtMaxRad = inputs.positionRad;
            operatorDashboard.turretHomingHalfDoneAlert.set(true);
            return;
        }

        // abs() so it doesn't matter which way the encoder counts
        double travelRad = Math.abs(homingAngleAtMaxRad - inputs.positionRad);

        if (travelRad < minimumPlausibleTravelRad) {
            // Almost certainly a double-press rather than a real second capture. Keep the first
            // point so the operator can just press again at the min stop.
            homingTravelAlert.set(true);
            return;
        }

        homingAngleAtMaxRad = null;
        setTravelRad(travelRad);

        // We're sitting at the min stop, so that is what the encoder reads right now
        io.setEncoderPosition(minPositionRad);
        // Start the profile from the seeded position, not the pre-seed reading inputs still hold
        goalState = new TrapezoidProfile.State(minPositionRad, 0.0);
        lookaheadState = goalState;

        homed = true;
        homingTravelAlert.set(false);
        operatorDashboard.turretNotHomedAlert.set(false);
        operatorDashboard.turretHomingHalfDoneAlert.set(false);
    }

    /** Centres the usable range on zero, so straight ahead is midway between the hard stops. */
    private void setTravelRad(double travelRad) {
        maxPositionRad = travelRad / 2.0;
        minPositionRad = -maxPositionRad;
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }

}
