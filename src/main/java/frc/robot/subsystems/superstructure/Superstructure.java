package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Util;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.superstructure.feeder.Feeder;
import frc.robot.subsystems.superstructure.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.spindexer.Spindexer;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.robotToCANrange;

public class Superstructure extends CommandBasedSubsystem {
    // https://v6.docs.ctr-electronics.com/en/stable/docs/application-notes/tuning-canrange.html
    private static final LoggedTunableNumber hasFuelThresholdMeters = new LoggedTunableNumber("Superstructure/HasFuelThresholdMeters", 0.33);
    private static final LoggedTunableNumber commitToShotThresholdMeters = new LoggedTunableNumber("Superstructure/CommitToShotThresholdMeters", 0.15);
    private static final LoggedTunableNumber commitToShotTimeSeconds = new LoggedTunableNumber("Superstructure/CommitToShotTimeSeconds", 0.1);

    private static final RobotState robotState = RobotState.get();
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    // because these subsystems are instantiated by Superstructure, instead of RobotContainer,
    // the variables shouldn't be static. Other singleton variables should be static, though
    public final Flywheel flywheel = Flywheel.get();
    public final Hood hood = Hood.get();
    public final Feeder feeder = Feeder.get();
    public final Spindexer spindexer = Spindexer.get();

    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE,
        SHOOT,
        EJECT,
        HOME_HOOD,
    }

    @Getter
    private Goal goal = Goal.IDLE;

    private boolean shouldGoalEnd() {
        if (goal == Goal.HOME_HOOD) {
            if (hood.isCurrentAtThresholdForHoming()) {
                hood.finishHoming();
                return true;
            }
        }
        return false;
    }

    public Command setGoal(Goal goal) {
        return startIdle(() -> this.goal = goal)
                .until(this::shouldGoalEnd);
    }

    private final Debouncer hasFuelDebouncer = new Debouncer(3.0, Debouncer.DebounceType.kFalling);
    private final Debouncer commitToShotDebouncer = new Debouncer(commitToShotTimeSeconds.get(), Debouncer.DebounceType.kFalling);

    private final Alert canrangeDisconnectedAlert = new Alert("CANrange is disconnected.", Alert.AlertType.kError);

    private static Superstructure instance;

    public static synchronized Superstructure get() {
        if (instance == null) {
            instance = new Superstructure();
        }

        return instance;
    }

    private Superstructure() {
        if (instance != null) {
            Util.error("Duplicate Superstructure created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure", inputs);

        canrangeDisconnectedAlert.set(!inputs.canrangeConnected);

        if (commitToShotTimeSeconds.hasChanged()) {
            commitToShotDebouncer.setDebounceTime(commitToShotTimeSeconds.get());
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Goal", goal);

        switch (goal) {
            case IDLE, HOME_HOOD -> {
                flywheel.setGoal(Flywheel.Goal.IDLE);
                feeder.setGoal(Feeder.Goal.IDLE);
                spindexer.setGoal(Spindexer.Goal.IDLE);
            }
            case SHOOT -> {
                if (operatorDashboard.disableCANrange.get() || hasFuelDebouncer.calculate(inputs.canrangeDistanceMeters < hasFuelThresholdMeters.get())) {
                    flywheel.setGoal(Flywheel.Goal.SHOOT);
                } else {
                    flywheel.setGoal(Flywheel.Goal.IDLE);
                }

                boolean shouldShoot = (
                        shootingKinematics.isShootingParametersMet() ||
                                commitToShotDebouncer.calculate(inputs.canrangeDistanceMeters < commitToShotThresholdMeters.get())
                ) || (
                        shootingKinematics.isShootingParametersMet() &&
                                operatorDashboard.disableCANrange.get()
                );
                if (shouldShoot) {
                    feeder.setGoal(Feeder.Goal.FEED);
                    spindexer.setGoal(Spindexer.Goal.FEED);
                } else {
                    feeder.setGoal(Feeder.Goal.IDLE);
                    spindexer.setGoal(Spindexer.Goal.IDLE);
                }
            }
            case EJECT -> {
                flywheel.setGoal(Flywheel.Goal.EJECT);
                feeder.setGoal(Feeder.Goal.EJECT);
                spindexer.setGoal(Spindexer.Goal.EJECT);
            }
        }

        if (goal == Goal.HOME_HOOD) {
            hood.setGoal(Hood.Goal.HOME);
        } else {
            hood.setGoal(Hood.Goal.SHOOT);
        }

        Logger.recordOutput(
                "Superstructure/FuelPose",
                new Pose3d(robotState.getPose())
                        .transformBy(robotToCANrange)
                        .transformBy(new Transform3d(
                                new Translation3d(inputs.canrangeDistanceMeters, 0.0, 0.0),
                                new Rotation3d()
                        ))
        );
    }
}
