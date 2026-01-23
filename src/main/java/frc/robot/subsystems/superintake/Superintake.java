package frc.robot.subsystems.superintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.superintake.intakepivot.IntakePivot;
import frc.robot.subsystems.superintake.intakerollers.IntakeRollers;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class Superintake extends CommandBasedSubsystem {
    private final RobotState robotState = RobotState.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    public final IntakeRollers intakeRollers = IntakeRollers.get();
    public final IntakePivot intakePivot = IntakePivot.get();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE,
        INTAKE,
        EJECT,
    }

    private Goal goal = Goal.IDLE;

    public Command setGoal(Goal goal) {
        return startIdle(() -> this.goal = goal);
    }

    private static Superintake instance;

    public static Superintake get() {
        if (instance == null)
            synchronized (Superintake.class) {
                instance = new Superintake();
            }

        return instance;
    }

    private Superintake() {
    }

    @Override
    public void periodicBeforeCommands() {
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superintake/Goal", goal);

        switch (goal) {
            case IDLE -> {
                intakeRollers.setGoal(IntakeRollers.Goal.IDLE);
                intakePivot.setGoal(IntakePivot.Goal.STOW);
            }
        }
    }
}
