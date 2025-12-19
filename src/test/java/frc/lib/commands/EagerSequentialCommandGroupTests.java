package frc.lib.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;

import static org.junit.jupiter.api.Assertions.*;

class EagerSequentialCommandGroupTests {
    @Test
    void testEagerness() {
        var ref = new Object() {
            boolean first = false;
            boolean second = false;
            boolean third = false;
            boolean fourth = false;
            boolean fifth = false;
            boolean sixth = false;
            boolean seventh = false;
        };
        EagerSequentialCommandGroup command = new EagerSequentialCommandGroup(
                Commands.runOnce(() -> ref.first = true),
                Commands.runOnce(() -> ref.second = true),
                // Requires 2 cycles
                new FunctionalCommand(
                        () -> {},
                        () -> {
                            if (ref.third) {
                                ref.fourth = true;
                            } else {
                                ref.third = true;
                            }
                        },
                        interrupted -> {},
                        () -> ref.fourth
                ),
                // Requires 2 cycles
                Commands.sequence(
                        Commands.runOnce(() -> ref.fifth = true),
                        // runOnce but it doesn't happen on init - needed since SequentialCommandGroup runs init when the previous command ends and we don't want that for this test
                        new FunctionalCommand(
                                () -> {},
                                () -> ref.sixth = true,
                                interrupted -> {},
                                () -> true
                        )
                ),
                // Requires 2 cycles
                Commands.sequence(
                        Commands.runOnce(() -> {}),
                        // runOnce but it doesn't happen on init - needed since SequentialCommandGroup runs init when the previous command ends and we don't want that for this test
                        new FunctionalCommand(
                                () -> {},
                                () -> ref.seventh = true,
                                interrupted -> {},
                                () -> true
                        )
                )
        );

        command.initialize();
        assertEquals(getCurrentCommandIndex(command), 0);

        command.execute();
        assertFalse(command.isFinished());
        assertEquals(getCurrentCommandIndex(command), 2);
        assertTrue(ref.first);
        assertTrue(ref.second);
        assertTrue(ref.third);
        assertFalse(ref.fourth);

        command.execute();
        assertFalse(command.isFinished());
        assertEquals(getCurrentCommandIndex(command), 3);
        assertTrue(ref.fourth);
        assertTrue(ref.fifth);
        assertFalse(ref.sixth);

        command.execute();
        assertFalse(command.isFinished());
        assertEquals(getCurrentCommandIndex(command), 4);
        assertTrue(ref.sixth);
        assertFalse(ref.seventh);

        command.execute();
        assertTrue(command.isFinished());
        assertEquals(getCurrentCommandIndex(command), 5);
        assertTrue(ref.seventh);
    }

    private int getCurrentCommandIndex(EagerSequentialCommandGroup commandGroup) {
        try {
            Field field = EagerSequentialCommandGroup.class.getDeclaredField("currentCommandIndex");
            field.setAccessible(true);
            return field.getInt(commandGroup);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }
}
