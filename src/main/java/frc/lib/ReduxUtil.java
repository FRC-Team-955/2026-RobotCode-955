package frc.lib;


import java.util.function.BooleanSupplier;

import static frc.lib.Util.asyncExecutor;

public class ReduxUtil {
    public static void tryUntilOk(int maxAttempts, BooleanSupplier command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.getAsBoolean();
            if (error) return;
        }
        Util.error("ReduxUtil tryUntilOk failure");
    }

    public static void tryUntilOkAsync(int maxAttempts, BooleanSupplier command) {
        asyncExecutor.execute(() -> tryUntilOk(maxAttempts, command));
    }
}
