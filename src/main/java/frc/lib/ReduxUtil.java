package frc.lib;


import java.util.function.Supplier;

import static frc.lib.Util.asyncExecutor;

public class ReduxUtil {
    public static void tryUntilOk(int maxAttempts, Supplier<Boolean> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error) return;
        }
        System.out.println("ReduxUtil tryUntilOk failure");
    }

    public static void tryUntilOkAsync(int maxAttempts, Supplier<Boolean> command) {
        asyncExecutor.execute(() -> tryUntilOk(maxAttempts, command));
    }
}
