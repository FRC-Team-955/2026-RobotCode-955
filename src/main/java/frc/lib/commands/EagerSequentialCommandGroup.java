package frc.lib.commands;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.List;

/** SequentialCommandGroup but commands that finish instantly don't take up a whole loop cycle */
/* package-private */ class EagerSequentialCommandGroup extends CommandComposition {
    private final List<Command> commands = new ArrayList<>();
    private int currentCommandIndex = -1;
    private boolean runWhenDisabled = true;
    private InterruptionBehavior interruptBehavior = InterruptionBehavior.kCancelIncoming;

    /**
     * Creates a new SequentialCommandGroup. The given commands will be run sequentially, with the
     * composition finishing when the last command finishes.
     *
     * @param commands the commands to include in this composition.
     */
    public EagerSequentialCommandGroup(Command... commands) {
        addCommands(commands);
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add, in order of execution.
     */
    @Override
    public void addCommands(Command... commands) {
        if (currentCommandIndex != -1) {
            throw new IllegalStateException("Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            this.commands.add(command);
            addRequirements(command.getRequirements());
            runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public final void initialize() {
        currentCommandIndex = 0;

        if (!commands.isEmpty()) {
            commands.get(0).initialize();
        }
    }

    @Override
    public final void execute() {
        if (commands.isEmpty()) {
            return;
        }

        Command currentCommand = commands.get(currentCommandIndex);

        currentCommand.execute();
        // If the current command is finished
        boolean endCurrentCommand = currentCommand.isFinished();
        while (endCurrentCommand) {
            endCurrentCommand = false;

            // End it
            currentCommand.end(false);
            currentCommandIndex++;

            // If we have another, move onto the next one
            if (currentCommandIndex < commands.size()) {
                currentCommand = commands.get(currentCommandIndex);
                currentCommand.initialize();
                currentCommand.execute();

                // Now end it again if it's already finished
                endCurrentCommand = currentCommand.isFinished();
            }
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (interrupted
                && !commands.isEmpty()
                && currentCommandIndex > -1
                && currentCommandIndex < commands.size()) {
            commands.get(currentCommandIndex).end(true);
        }
        currentCommandIndex = -1;
    }

    @Override
    public final boolean isFinished() {
        return currentCommandIndex == commands.size();
    }

    @Override
    public boolean runsWhenDisabled() {
        return runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return interruptBehavior;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addIntegerProperty("index", () -> currentCommandIndex, null);
    }
}