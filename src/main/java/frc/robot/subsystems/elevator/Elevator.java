package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.ReefAlign;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.RobotMechanism.middleOfRobot;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.elevator.ElevatorTuning.*;

public class Elevator implements Periodic {
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final ElevatorIO io = createIO();
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public enum RequestType {
        VoltageVolts,
        PositionMeters,
    }

    @RequiredArgsConstructor
    public enum Goal {
        STOW(stowGoalSetpoint::get, RequestType.PositionMeters, false), // Setpoint for when coral stuck in robot mode is activated is in periodicAfterCommands
        SCORE_L1(scoreL1GoalSetpoint::get, RequestType.PositionMeters, false),
        SCORE_L2(scoreL2GoalSetpoint::get, RequestType.PositionMeters, true),
        SCORE_L3(scoreL3GoalSetpoint::get, RequestType.PositionMeters, true),
        SCORE_L4(scoreL4GoalSetpoint::get, RequestType.PositionMeters, true),
        DESCORE_L2(descoreL2GoalSetpoint::get, RequestType.PositionMeters, false),
        DESCORE_L3(descoreL3GoalSetpoint::get, RequestType.PositionMeters, false),
        ZERO_CORAL(() -> -0.7, RequestType.VoltageVolts, false),
        ;

        /** Should be constant for every loop cycle */
        public final DoubleSupplier value;
        public final RequestType type;
        private final boolean adjustForScoring;
    }

    @Getter
    @Setter
    private Goal goal = Goal.STOW;

    @SuppressWarnings({"FieldMayBeFinal", "FieldCanBeLocal"}) // TODO: remove when zeroing added
    @AutoLogOutput(key = "Elevator/HasZeroed")
    private boolean hasZeroed = false;

    private boolean autoStop = false;
    private final Timer autoStopTimer = new Timer();
    private boolean prevEmergencyStopped = false;

    /** NOTE: UNITS IN METERS! */
    private TrapezoidProfile profileFullVelocity = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            maxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSquared
    ));
    private TrapezoidProfile profileGentleVelocity = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            gentleMaxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSquared
    ));
    private TrapezoidProfile.State previousStateMeters = null;

    @AutoLogOutput(key = "Elevator/DistanceFromScoringPositionMeters")
    private double distanceFromScoringPositionMeters = 0.0;

    private boolean manualCurrentLimitApplied = false;

    private final Alert emergencyStoppedAlert = new Alert("Elevator is emergency stopped.", Alert.AlertType.kError);
    private final Alert notZeroedAlert = new Alert("Elevator is not zeroed! Please zero.", Alert.AlertType.kError);
    private final Alert leaderDisconnectedAlert = new Alert("Elevator leader motor is disconnected.", Alert.AlertType.kError);
    private final Alert followerDisconnectedAlert = new Alert("Elevator follower motor is disconnected.", Alert.AlertType.kError);
    private final Alert offsetSetAlert = new Alert("Elevator offset is not zero, bad things may happen.", Alert.AlertType.kWarning);
    private final Alert temperatureAlert = new Alert("Elevator motor temperature is high.", Alert.AlertType.kWarning);

    private static Elevator instance;

    public static Elevator get() {
        if (instance == null)
            synchronized (Elevator.class) {
                instance = new Elevator();
            }

        return instance;
    }

    private Elevator() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Elevator", inputs);

        leaderDisconnectedAlert.set(!inputs.leaderConnected);
        followerDisconnectedAlert.set(!inputs.followerConnected);

        temperatureAlert.set(Math.max(inputs.leaderTemperatureCelsius, inputs.followerTemperatureCelsius) > 60);

        // Check emergency stop and limits for auto stop
        var positionMeters = getPositionMeters();
        var velocityMetersPerSec = getVelocityMetersPerSec();
        if (!autoStop) {
            autoStop =
                    (positionMeters > upperLimit.positionMeters()
                            && velocityMetersPerSec > upperLimit.velocityMetersPerSec()
                    ) || (positionMeters < lowerLimit.positionMeters()
                            && velocityMetersPerSec < lowerLimit.velocityMetersPerSec());

            if (autoStop) {
                autoStopTimer.restart();
            }
        } else if (autoStopTimer.hasElapsed(0.75) && Math.abs(velocityMetersPerSec) < 0.5) {
            // Only disable auto stop if we have stopped for a bit (roughly - we don't want to get stuck in auto stop)
            autoStop = false;
        }
        Logger.recordOutput("Elevator/AutoStop", autoStop);

        boolean emergencyStopped = operatorDashboard.elevatorEStop.get() || autoStop;
        if (emergencyStopped != prevEmergencyStopped) {
            if (emergencyStopped) {
                System.out.println("Elevator is emergency stopping");
            } else {
                System.out.println("Elevator is no longer emergency stopped");
            }
            io.setEmergencyStopped(emergencyStopped);
            prevEmergencyStopped = emergencyStopped;
        }
        emergencyStoppedAlert.set(emergencyStopped);
        Logger.recordOutput("Elevator/EmergencyStop", emergencyStopped);

        // Update mechanisms
        robotMechanism.elevator.stage1Root.setPosition(middleOfRobot - Units.inchesToMeters(7) + 0.04, Units.inchesToMeters(2.85) + getPositionMeters() / 3);
        robotMechanism.elevator.stage2Root.setPosition(middleOfRobot - Units.inchesToMeters(7) + 0.02, Units.inchesToMeters(3.85) + getPositionMeters() / 3 * 2);
        robotMechanism.elevator.stage3Root.setPosition(middleOfRobot - Units.inchesToMeters(7), Units.inchesToMeters(4.85) + getPositionMeters());

        var endEffectorX = middleOfRobot - Units.inchesToMeters(11);
        var endEffectorY = Units.inchesToMeters(7) + getPositionMeters();
        robotMechanism.endEffector.root.setPosition(endEffectorX, endEffectorY);
        robotMechanism.endEffector.topRollersRoot.setPosition(endEffectorX - Units.inchesToMeters(3), endEffectorY + Units.inchesToMeters(10));

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        gainsTunable.ifChanged(io::setPIDF);

        if (maxVelocityMetersPerSecondTunable.hasChanged()
                || maxAccelerationMetersPerSecondSquaredTunable.hasChanged()
        ) {
            profileFullVelocity = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    maxVelocityMetersPerSecondTunable.get(),
                    maxAccelerationMetersPerSecondSquaredTunable.get()
            ));
            profileGentleVelocity = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    gentleMaxVelocityMetersPerSecond,
                    maxAccelerationMetersPerSecondSquaredTunable.get()
            ));
            hardstopSlowdownMeters = calculateHardstopSlowdownMeters(maxVelocityMetersPerSecondTunable.get());
            robotMechanism.elevator.updateHardstopSlowdownPosition();
        }
    }

    @Override
    public void periodicAfterCommands() {
        // Update current limit
        if (true) {//operatorDashboard.manualElevator.get() || goal == Goal.ZERO_ELEVATOR) {
            if (!manualCurrentLimitApplied) {
                io.setManualCurrentLimit(true);
                manualCurrentLimitApplied = true;
            }
//            throw new RuntimeException("TODO");
        } else {
            if (manualCurrentLimitApplied) {
                io.setManualCurrentLimit(false);
                manualCurrentLimitApplied = false;
            }
        }

        // Handle goal
        Logger.recordOutput("Elevator/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setVoltage(0);
            previousStateMeters = null;
        } else {
            Logger.recordOutput("Elevator/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("Elevator/RequestValue", value);
            switch (goal.type) {
                case VoltageVolts -> io.setVoltage(value);
                case PositionMeters -> {
                    double positionMeters = getPositionMeters();
                    double velocityMetersPerSec = getVelocityMetersPerSec();

                    double setpointMeters = value;
                    if (goal.adjustForScoring) {
                        setpointMeters += calculatePositionOffsetForScoring();
                    }
                    setpointMeters = MathUtil.clamp(setpointMeters, 0, maxHeightMeters);

                    double offsetMeters = operatorDashboard.elevatorOffsetMeters.get();
                    if (offsetMeters != 0.0) {
                        setpointMeters += offsetMeters; // Offset should override clamping
                        offsetSetAlert.set(true);
                    } else {
                        offsetSetAlert.set(false);
                    }

                    boolean usingGentleVelocity = (velocityMetersPerSec < 0 || setpointMeters + 0.1 < positionMeters) // If we are going down
                            // If we are below the hardstop slowdown zone
                            && positionMeters < hardstopSlowdownMeters;
                    // Only actually use the gentle profile if we are close enough to the max velocity to avoid jumping directly to max velocity
                    boolean usingGentleProfile = usingGentleVelocity && Math.abs(velocityMetersPerSec) < gentleMaxVelocityMetersPerSecond + 0.4;

                    if (goal == Goal.STOW && operatorDashboard.coralStuckInRobotMode.get()) {
                        // Override stow setpoint if coral is stuck in the robot
                        setpointMeters = 1.1;
                        // Use gentle so we don't slam coral into one of the crossbars
                        usingGentleProfile = true;
                    }

                    var profile = usingGentleProfile
                            ? profileGentleVelocity
                            : profileFullVelocity;
                    // If not using the gentle profile, set the setpoint to the hardstop with the gentle max velocity
                    TrapezoidProfile.State setpointState = usingGentleVelocity && !usingGentleProfile
                            ? new TrapezoidProfile.State(hardstopMeters, gentleMaxVelocityMetersPerSecond)
                            : new TrapezoidProfile.State(setpointMeters, 0);

                    // Sometimes the profile outruns the elevator, so failsafe if it does
                    var usingRealStateAsCurrent = operatorDashboard.useRealElevatorState.get();
                    if (usingRealStateAsCurrent) {
                        // Turn the toggle off instantly so it's like a button
                        // We only want to use the real state for one cycle anyways
                        operatorDashboard.useRealElevatorState.set(false);
                    }
                    var currentState = previousStateMeters == null || usingRealStateAsCurrent
                            ? new TrapezoidProfile.State(positionMeters, velocityMetersPerSec)
                            : previousStateMeters;

                    previousStateMeters = profile.calculate(0.02, currentState, setpointState);

                    var setpointPositionRad = metersToRad(previousStateMeters.position);
                    var setpointVelocityRadPerSec = metersToRad(previousStateMeters.velocity);

                    io.setMotionProfile(setpointPositionRad, setpointVelocityRadPerSec);

                    Logger.recordOutput("Elevator/UsingGentleProfile", usingGentleProfile);
                    Logger.recordOutput("Elevator/UsingRealStateAsCurrent", usingRealStateAsCurrent);

                    Logger.recordOutput("Elevator/Profile/PositionMeters", previousStateMeters.position);
                    Logger.recordOutput("Elevator/Profile/VelocityMetersPerSec", previousStateMeters.velocity);
                }
            }
        }

        // Update zeroed alert - after commands, since that would be when it gets zeroed
        notZeroedAlert.set(!hasZeroed);
    }

    @AutoLogOutput(key = "Elevator/AtGoal")
    private boolean atGoal() {
        // if goal.setpointMeters is null, will be false and won't crash
        return goal.value != null
                && Math.abs(goal.value.getAsDouble() - getPositionMeters()) <= setpointPositionToleranceMeters
                && Math.abs(getVelocityMetersPerSec()) <= setpointVelocityToleranceMetersPerSec;
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }

    @AutoLogOutput(key = "Elevator/Measurement/PositionMeters")
    public double getPositionMeters() {
        return radToMeters(inputs.leaderPositionRad);
    }

    @AutoLogOutput(key = "Elevator/Measurement/VelocityMetersPerSec")
    public double getVelocityMetersPerSec() {
        return radToMeters(inputs.leaderVelocityRadPerSec);
    }

    public Command setDistanceFromScoringPositionContinuous(DoubleSupplier distanceFromScoringPositionMeters) {
        // Don't require subsystem - meant to run in background
        return Commands.runEnd(
                () -> this.distanceFromScoringPositionMeters = distanceFromScoringPositionMeters.getAsDouble(),
                () -> this.distanceFromScoringPositionMeters = 0.0
        );
    }

    private double calculatePositionOffsetForScoring() {
        // Safeguard - this shouldn't happen due to when we set the distance but you never know
        if (distanceFromScoringPositionMeters > ReefAlign.alignLinearToleranceMeters) {
            return MathUtil.clamp(distanceFromScoringPositionMeters, 0, 0.5) * positionOffsetPerMeterOfDistance;
        } else {
            return 0.0;
        }
    }

    public double getDriveConstraintScalar() {
        double elevatorSetpoint = goal.value != null
                ? goal.value.getAsDouble()
                : 0;
        double elevatorPosition = Math.max(getPositionMeters(), elevatorSetpoint);
        return MathUtil.interpolate(
                1,
                DriveConstants.constraintScalarWhenElevatorAtMaxHeightDriver,
                elevatorPosition / maxHeightMeters
        );
    }

    public Command zeroCoral() {
        throw new RuntimeException("TODO");
//        return CommandsExt.eagerSequence(
//                setGoal(() -> Goal.ZERO_CORAL),
//                startEndWaitUntil(
//                        () -> io.setOpenLoop(-0.7),
//                        () -> io.setOpenLoop(0),
//                        () -> getPositionMeters() < 0.01
//                ),
//                Commands.idle()
//        );
    }

    public Command zeroElevator() {
        throw new RuntimeException("TODO");
//        var elevatorInitialPosition = new Object() {
//            double val = 0.0;
//        };
//        return Commands.either(
//                CommandsExt.eagerSequence(
//                        setGoal(() -> Goal.ZERO_ELEVATOR),
//                        runOnce(() -> io.setOpenLoop(-0.5)),
//                        Commands.waitSeconds(0.2),
//                        Commands.waitUntil(() -> getVelocityMetersPerSec() < 0.02),
//                        runOnce(() -> {
//                            elevatorInitialPosition.val = getPositionMeters();
//                            io.setOpenLoop(0.0);
//                        }),
//                        Commands.waitSeconds(1),
//                        runOnce(() -> {
//                            if (Math.abs(getPositionMeters() - elevatorInitialPosition.val) < 0.02) {
//                                io.setEncoder(0);
//                                hasZeroed = true;
//                            }
//                        })
//                ).ignoringDisable(false).asProxy(), // Notice the proxy - it is important
//                Commands.runOnce(() -> {
//                    io.setEncoder(0);
//                    hasZeroed = true;
//                }).ignoringDisable(true),
//                DriverStation::isEnabled
//        );
    }

    public Command setManualVoltage(double addedVoltage) {
        throw new RuntimeException("TODO JOYSTICK MANUAL CONTROL");
        // Note - doesn't require subsystem to allow other commands that would require elevator to work
//        return Commands.startEnd(
//                () -> manualVoltage = gains.kG() + addedVoltage,
//                () -> manualVoltage = gains.kG()
//        );
    }
}