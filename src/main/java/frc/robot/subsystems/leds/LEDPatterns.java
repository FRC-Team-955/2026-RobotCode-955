package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

/**
 * https://github.com/FRC-Team-955/2025-RobotCode-955/blob/affb12cb55955c40cd5e10fe432037460ca2f039/src/main/java/frc/robot/subsystems/leds/LEDPatterns.java
 */
public class LEDPatterns {
    /**
     * LEDPattern.breathe uses RobotController.getTime(), which doesn't really
     * work when the robot hasn't started up yet. Therefore, we must implement
     * the breathe algorithm ourselves. This is based off of 749's 2024 code.
     */
    public static LEDPattern startup() {
        Color color1 = Color.kRed;
        Color color2 = Color.kBlack;
        double period = 1.0;

        return (reader, writer) -> {
            double timestamp = System.currentTimeMillis() / 1000.0;
            double percent = timestamp % period / (period / 2.0);
            if (percent > 1) {
                percent = 1 - (percent - 1);
            }

            double interp = (Math.tanh((percent - 0.5) * 3.0) / Math.tanh(0.5 * 3.0)) * 0.5 + 0.5;

            double r = color2.red + (color1.red - color2.red) * interp;
            double g = color2.green + (color1.green - color2.green) * interp;
            double b = color2.blue + (color1.blue - color2.blue) * interp;

            Color color = new Color(r, g, b);
            for (int i = 0; i < reader.getLength(); i++) {
                writer.setLED(i, color);
            }
        };
    }

    public static LEDPattern autoPlacementProgress(DoubleSupplier progressSupplier) {
        return LEDPattern.solid(Color.kGreen).mask(LEDPattern.progressMaskLayer(progressSupplier));
    }

    // All modes
    public static final LEDPattern lowBattery = LEDPattern.solid(Color.kRed).blink(Seconds.of(1.0 / 3.0));

    // Disabled
    public static final LEDPattern autoNotChosen = LEDPattern.solid(Color.kBlue).blink(Seconds.of(1));
    public static final LEDPattern badAutoPlacement = LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.5));
    public static final LEDPattern visionDisconnected = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.5));
    public static final LEDPattern autoReady = LEDPattern.gradient(
            LEDPattern.GradientType.kContinuous,
            Color.kRed,
            new Color(255, 0, 128)
    ).scrollAtRelativeSpeed(Hertz.of(2));

    // Enabled
    public static final LEDPattern eject = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.1));
    public static final LEDPattern aiming = LEDPattern.solid(Color.kYellow);
    public static final LEDPattern shooting = LEDPattern.solid(Color.kGreen);
    public static final LEDPattern intaking = LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.1));
    public static final LEDPattern waitingForShift = LEDPattern.solid(Color.kPink).blink(Seconds.of(0.1));
    public static final LEDPattern homing = LEDPattern.solid(Color.kBlue);
    public static final LEDPattern idle = LEDPattern.kOff;
    public static final LEDPattern hubSwitch = LEDPattern.gradient(
            LEDPattern.GradientType.kContinuous,
            Color.kRed,
            Color.kBlue
    ).scrollAtRelativeSpeed(Hertz.of(2));
}
