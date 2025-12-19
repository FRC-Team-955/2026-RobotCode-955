package frc.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class CommandComposition extends Command {
    public abstract void addCommands(Command... commands);
}
