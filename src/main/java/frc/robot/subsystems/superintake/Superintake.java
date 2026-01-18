package frc.robot.subsystems.superintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.CommandsExt;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class Superintake extends CommandBasedSubsystem {
    private final RobotState robotState = RobotState.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final AprilTagVision aprilTagVision = AprilTagVision.get();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE,
    }

    private Goal goal = Goal.IDLE;

    public Command setGoal(Goal superintakeGoal) {
        return runOnce(() -> {
            goal = superintakeGoal;
        });
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
    }

    public Command cancel() {
        return CommandsExt.eagerSequence(
                setGoal(
                        Goal.IDLE
                ),
                aprilTagVision.setTagIdFilter(new int[0])
        ).ignoringDisable(true);
    }
}