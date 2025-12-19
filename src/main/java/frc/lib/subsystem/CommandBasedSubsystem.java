package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.commands.CommandsExt;

import java.util.function.BooleanSupplier;

public abstract class CommandBasedSubsystem extends SubsystemBase implements Periodic {
    @Override
    // did you know you can have final methods in java? neither did I
    public final void periodic() {
        // periodicBeforeCommands is run by Robot
    }

    public Command waitUntil(BooleanSupplier isFinished) {
        return CommandsExt.waitUntilRequirements(isFinished, this);
    }

    public Command runOnceAndWaitUntil(Runnable initialize, BooleanSupplier isFinished) {
        return CommandsExt.runOnceAndWaitUntil(initialize, isFinished, this);
    }

    public Command startIdle(Runnable initialize) {
        return CommandsExt.startIdle(initialize, this);
    }

    public Command startIdleWaitUntil(Runnable initialize, BooleanSupplier isFinished) {
        return CommandsExt.startIdleWaitUntil(initialize, isFinished, this);
    }

    public Command startEndWaitUntil(Runnable initialize, Runnable end, BooleanSupplier isFinished) {
        return CommandsExt.startEndWaitUntil(initialize, end, isFinished, this);
    }
}
