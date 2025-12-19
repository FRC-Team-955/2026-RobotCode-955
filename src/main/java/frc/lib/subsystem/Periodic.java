package frc.lib.subsystem;

public interface Periodic {
    default void periodicBeforeCommands() {
    }

    default void periodicAfterCommands() {
    }
}
