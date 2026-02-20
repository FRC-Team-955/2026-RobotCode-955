package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Util;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.ShootingKinematics;
import frc.robot.subsystems.superstructure.feeder.Feeder;
import frc.robot.subsystems.superstructure.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.flywheel.FlywheelIO.FlywheelCurrentLimitMode;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.spindexer.Spindexer;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;

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
    }

    @Getter
    private Goal goal = Goal.IDLE;

    public Command setGoal(Goal goal) {
        return startIdle(() -> this.goal = goal);
    }

    public Command setGoal(Supplier<Goal> goal) {
        return run(() -> this.goal = goal.get());
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
                switch (operatorDashboard.getSelectedScoringMode()) {
                    case ShootAndPassAutomatic -> {
                        flywheel.setGoal(Flywheel.Goal.SHOOT_AND_PASS_AUTOMATIC);
                        hood.setGoal(Hood.Goal.SHOOT_AND_PASS_AUTOMATIC);
                    }
                    case ShootHubManual -> {
                        flywheel.setGoal(Flywheel.Goal.SHOOT_HUB_MANUAL);
                        hood.setGoal(Hood.Goal.SHOOT_HUB_MANUAL);
                    }
                    case ShootTowerManual -> {
                        flywheel.setGoal(Flywheel.Goal.SHOOT_TOWER_MANUAL);
                        hood.setGoal(Hood.Goal.SHOOT_TOWER_MANUAL);
                    }
                    case PassManual -> {
                        flywheel.setGoal(Flywheel.Goal.PASS_MANUAL);
                        hood.setGoal(Hood.Goal.PASS_MANUAL);
                    }
                }
                if (goal == Goal.SHOOT && shootingKinematics.isShootingParametersMet()) {
                    flywheel.setCurrentLimitMode(FlywheelCurrentLimitMode.SHOOT);
                    feeder.setGoal(Feeder.Goal.FEED);
                    spindexer.setGoal(Spindexer.Goal.FEED);
                } else {
                    flywheel.setCurrentLimitMode(FlywheelCurrentLimitMode.SPINUP);
                    feeder.setGoal(Feeder.Goal.IDLE);
                    spindexer.setGoal(Spindexer.Goal.IDLE);
                }
            }
            case EJECT -> {
                flywheel.setGoal(Flywheel.Goal.EJECT);
                flywheel.setCurrentLimitMode(FlywheelCurrentLimitMode.SHOOT);
                hood.setGoal(Hood.Goal.EJECT);
                feeder.setGoal(Feeder.Goal.EJECT);
                spindexer.setGoal(Spindexer.Goal.EJECT);
            }
        }
    }
}
