package frc.lib.subsystem;

public interface Periodic {
    default void updateAndProcessInputs() {
    }

    default void periodicBeforeCommands() {
    }

    default void periodicAfterCommands() {
    }
}
