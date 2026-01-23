package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.superstructure.indexer.Indexer;
import frc.robot.subsystems.superstructure.flywheel.Flywheel;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.createIO;

public class Superstructure extends CommandBasedSubsystem {
    private final RobotState robotState = RobotState.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    public final Indexer indexer = Indexer.get();
    public final Flywheel flywheel = Flywheel.get();

    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE,
    }

    private Goal goal = Goal.IDLE;

    public Command setGoal(Goal goal) {
        return startIdle(() -> this.goal = goal);
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

            }
        }
    }

    public Command runIndexer() {
        return indexer.setGoal(
                Indexer.Goal.RUN_AT_VOLTAGE
        );
    }

    public Command runFlywheel() {
        return flywheel.setGoal(
                    Flywheel.Goal.RUN_AT_CERTAIN_SPEED
        );
    }
}