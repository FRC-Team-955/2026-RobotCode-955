package frc.robot.subsystems.superintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Util;
import frc.lib.commands.CommandsExt;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.subsystems.superintake.intakepivot.IntakePivot;
import frc.robot.subsystems.superintake.intakerollers.IntakeRollers;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class Superintake extends CommandBasedSubsystem {
    // because these subsystems are instantiated by Superintake, instead of RobotContainer,
    // the variables shouldn't be static. Other singleton variables should be static, though
    public final IntakePivot intakePivot = IntakePivot.get();
    public final IntakeRollers intakeRollers = IntakeRollers.get();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE,
        DEPLOY,
        INTAKE,
        SHOOT,
        EJECT,
        HOME_INTAKE_PIVOT,
    }

    @Getter
    private Goal goal = Goal.IDLE;

    private boolean shouldGoalEnd() {
        if (goal == Goal.HOME_INTAKE_PIVOT) {
            if (intakePivot.isCurrentAtThresholdForHoming()) {
                intakePivot.finishHoming();
                return true;
            }
        }
        return false;
    }

    public Command setGoal(Goal goal) {
        return startIdle(() -> this.goal = goal)
                .until(this::shouldGoalEnd);
    }

    private static Superintake instance;

    public static synchronized Superintake get() {
        if (instance == null) {
            instance = new Superintake();
        }

        return instance;
    }

    private Superintake() {
        if (instance != null) {
            Util.error("Duplicate Superintake created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superintake/Goal", goal);

        switch (goal) {
            case IDLE -> {
                intakePivot.setGoal(IntakePivot.Goal.STOW);
                intakeRollers.setGoal(IntakeRollers.Goal.IDLE);
            }
            case DEPLOY -> {
                intakePivot.setGoal(IntakePivot.Goal.DEPLOY);
                intakeRollers.setGoal(IntakeRollers.Goal.IDLE);
            }
            case INTAKE -> {
                intakePivot.setGoal(IntakePivot.Goal.DEPLOY);
                intakeRollers.setGoal(IntakeRollers.Goal.INTAKE);
            }
            case SHOOT -> {
                intakePivot.setGoal(IntakePivot.Goal.STOW);
                intakeRollers.setGoal(IntakeRollers.Goal.INTAKE);
            }
            case EJECT -> {
                intakePivot.setGoal(IntakePivot.Goal.DEPLOY);
                intakeRollers.setGoal(IntakeRollers.Goal.EJECT);
            }
            case HOME_INTAKE_PIVOT -> {
                intakePivot.setGoal(IntakePivot.Goal.HOME);
                intakeRollers.setGoal(IntakeRollers.Goal.IDLE);
            }
        }
    }

    public Command intakeShootAlternate() {
        return CommandsExt.repeatingEagerSequence(
                setGoal(Goal.INTAKE).withTimeout(1),
                setGoal(Goal.SHOOT).withTimeout(1)
        );
    }
}
