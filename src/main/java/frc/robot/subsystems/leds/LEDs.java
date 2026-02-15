package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.subsystem.Periodic;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static frc.robot.subsystems.leds.LEDConstants.*;

public class LEDs implements Periodic {
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Superintake superintake = Superintake.get();
    private final Superstructure superstructure = Superstructure.get();

    // See createAndStartStartupNotifier for why this is static
    private static final LEDsIO io = createIO();

    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
    private final AddressableLEDBufferView firstHalfView = new AddressableLEDBufferView(buffer, 0, length / 2 - 1);
    private final AddressableLEDBufferView secondHalfView = new AddressableLEDBufferView(buffer, length / 2, length - 1);

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.5, 2.1, new Color8Bit(Color.kBlack));
    private final LoggedMechanismLigament2d[] ligaments = new LoggedMechanismLigament2d[length];

    private static LEDs instance;

    public static LEDs get() {
        synchronized (LEDs.class) {
            if (instance == null) {
                instance = new LEDs();
            }
        }

        return instance;
    }

    private LEDs() {
        double bottomY = 0.5;
        double middleOfRobot = 1.5 / 2.0;
        for (int index = 0; index < length; index++) {
            String name = "LED" + index;
            LoggedMechanismRoot2d root = mechanism.getRoot(
                    name,
                    middleOfRobot,
                    bottomY + index * 0.1
            );
            ligaments[index] = root.append(new LoggedMechanismLigament2d(
                    name,
                    0.1,
                    90,
                    10,
                    new Color8Bit(Color.kBlack)
            ));
        }
    }

    /**
     * If we don't make this static, LEDs will need to be instantiated, which means that
     * Superintake, Superstructure, etc etc needs to be instantiated as well.
     */
    public static Notifier createAndStartStartupNotifier() {
        LEDPattern pattern = LEDPatterns.startup();
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
        Runnable callback = () -> {
            pattern.applyTo(buffer);
            io.setData(buffer);
        };
        callback.run(); // run it once, before the notifier starts
        Notifier notifier = new Notifier(callback);
        notifier.startPeriodic(Constants.loopPeriod);
        return notifier;
    }

    @Override
    public void periodicAfterCommands() {
        boolean lowBattery = operatorDashboard.isBatteryVoltageAlertActive();
        boolean cameraError = AprilTagVision.get().anyCamerasDisconnected() || GamePieceVision.get().anyCamerasDisconnected();

        if (lowBattery) {
            LEDPatterns.lowBattery.applyTo(buffer);
        } else if (cameraError) {
            LEDPatterns.visionDisconnected.applyTo(buffer);
        } else if (DriverStation.isDisabled()) {
            LEDPatterns.autoReady.applyTo(buffer);
        } else if (DriverStation.isEnabled()) {
            boolean endgame = DriverStation.isTeleop() &&
                    DriverStation.getMatchTime() > endgameLowerThresholdSeconds &&
                    DriverStation.getMatchTime() < endgameUpperThresholdSeconds;
            if (endgame) {
                LEDPatterns.endgame.applyTo(buffer);
            } else {
                LEDPattern superintakePattern = switch (superintake.getGoal()) {
                    case IDLE -> null;
                    case INTAKE -> LEDPatterns.intaking;
                    case EJECT -> LEDPatterns.eject;
                    case AUTO_INTAKE_INTAKING -> null;
                    case AUTO_INTAKE_SEARCHING_FOR_STALE -> null;

                    case AUTO_INTAKE_SEARCHING -> null;

                };

                LEDPattern superstructurePattern = switch (superstructure.getGoal()) {
                    case IDLE, SPINUP -> null;
                    case SHOOT -> LEDPatterns.shooting;
                    case EJECT -> LEDPatterns.eject;

                };

                if (superintakePattern != null && superstructurePattern != null) {
                    superintakePattern.applyTo(firstHalfView);
                    superstructurePattern.applyTo(secondHalfView);
                } else if (superintakePattern != null) {
                    superintakePattern.applyTo(buffer);
                } else if (superstructurePattern != null) {
                    superstructurePattern.applyTo(buffer);
                } else {
                    LEDPatterns.idle.applyTo(buffer);
                }
            }
        }

        io.setData(buffer);

        // Update mechanism
        for (int i = 0; i < buffer.getLength(); i++) {
            // https://github.com/FRC-Team-955/2024-RobotCode-749/blob/kotlin-old/src/main/java/frc/robot/subsystems/leds/LEDs.kt#L88
//            System.out.print("\u001b[38;2;" + buffer.getRed(i) + ";" + buffer.getGreen(i) + ";" + buffer.getBlue(i) + "m■\u001b[0m");
            ligaments[i].setColor(new Color8Bit(
                    buffer.getRed(i),
                    buffer.getGreen(i),
                    buffer.getBlue(i)
            ));
        }
//        System.out.println();
        Logger.recordOutput("LEDs/Mechanism", mechanism);
    }
}
