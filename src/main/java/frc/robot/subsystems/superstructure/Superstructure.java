package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Util;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.ShootingKinematics;
import frc.robot.subsystems.superstructure.feeder.Feeder;
import frc.robot.subsystems.superstructure.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.spindexer.Spindexer;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.robotToCANrange;

public class Superstructure extends CommandBasedSubsystem {
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
        SPINUP,
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

    public Command setGoal(Supplier<Goal> goal) {
        return run(() -> this.goal = goal.get())
                .until(this::shouldGoalEnd);
    }

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
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Goal", goal);

        switch (goal) {
            case IDLE -> {
                flywheel.setGoal(Flywheel.Goal.IDLE);
                hood.setGoal(Hood.Goal.STOW);
                feeder.setGoal(Feeder.Goal.IDLE);
                spindexer.setGoal(Spindexer.Goal.IDLE);
            }
            case SPINUP, SHOOT -> {
                flywheel.setGoal(Flywheel.Goal.SHOOT);
                hood.setGoal(Hood.Goal.SHOOT);

                if (goal == Goal.SHOOT && shootingKinematics.isShootingParametersMet()) {
//                    flywheel.setCurrentLimitMode(FlywheelCurrentLimitMode.SHOOT);
                    feeder.setGoal(Feeder.Goal.FEED);
                    spindexer.setGoal(Spindexer.Goal.FEED);
                } else {
//                    flywheel.setCurrentLimitMode(FlywheelCurrentLimitMode.SPINUP);
                    feeder.setGoal(Feeder.Goal.IDLE);
                    spindexer.setGoal(Spindexer.Goal.IDLE);
                }
            }
            case EJECT -> {
                flywheel.setGoal(Flywheel.Goal.EJECT);
//                flywheel.setCurrentLimitMode(FlywheelCurrentLimitMode.SHOOT);
                hood.setGoal(Hood.Goal.EJECT);
                feeder.setGoal(Feeder.Goal.EJECT);
                spindexer.setGoal(Spindexer.Goal.EJECT);
            }
            case HOME_HOOD -> {
                flywheel.setGoal(Flywheel.Goal.IDLE);
                hood.setGoal(Hood.Goal.HOME);
                feeder.setGoal(Feeder.Goal.IDLE);
                spindexer.setGoal(Spindexer.Goal.IDLE);
            }
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
