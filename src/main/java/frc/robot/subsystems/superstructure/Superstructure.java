package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.spindexer.Spindexer;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;

public class Superstructure extends CommandBasedSubsystem {
    private final RobotState robotState = RobotState.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    public final Flywheel flywheel = Flywheel.get();
    public final Hood hood = Hood.get();
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

    public static Superstructure get() {
        if (instance == null)
            synchronized (Superstructure.class) {
                instance = new Superstructure();
            }

        return instance;
    }

    private Superstructure() {
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
                switch (goal) {
                    case SPINUP -> spindexer.setGoal(Spindexer.Goal.IDLE);
                    case SHOOT -> spindexer.setGoal(Spindexer.Goal.FEED);
                }
            }
            case EJECT -> {
                flywheel.setGoal(Flywheel.Goal.EJECT);
                hood.setGoal(Hood.Goal.EJECT);
                spindexer.setGoal(Spindexer.Goal.EJECT);
            }
        }
    }
}
