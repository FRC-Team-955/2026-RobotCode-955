package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.HubShiftTracker;
import frc.robot.OperatorDashboard;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static frc.robot.subsystems.leds.LEDConstants.createIO;
import static frc.robot.subsystems.leds.LEDConstants.length;

public class LEDs implements Periodic {
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    private static final AprilTagVision aprilTagVision = AprilTagVision.get();
    private static final GamePieceVision gamePieceVision = GamePieceVision.get();

    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    // See createAndStartStartupNotifier for why this is static
    private static final LEDsIO io = createIO();

    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
    private final int mid = length / 2;
    private final AddressableLEDBufferView leftHalfView = new AddressableLEDBufferView(buffer, 0, mid - 1);
    private final AddressableLEDBufferView rightHalfView = new AddressableLEDBufferView(buffer, mid, length - 1);

    private final int quarterSize = Math.max(1, mid / 2);
    private final AddressableLEDBufferView firstQuarterView = new AddressableLEDBufferView(buffer, 0, quarterSize - 1);
    private final AddressableLEDBufferView secondQuarterView = new AddressableLEDBufferView(buffer, quarterSize, mid - 1);

    private static final double HUB_BLINK_START_SECONDS = 5.0;
    private static final double HUB_BLINK_MIN_PERIOD = 0.5;
    private static final double HUB_BLINK_MAX_PERIOD = 1.5;

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.5, 2.1, new Color8Bit(Color.kBlack));
    private final LoggedMechanismLigament2d[] ligaments = new LoggedMechanismLigament2d[length];

    private static LEDs instance;

    public static synchronized LEDs get() {
        if (instance == null) {
            instance = new LEDs();
        }

        return instance;
    }

    private LEDs() {
        if (instance != null) {
            Util.error("Duplicate LEDs created");
        }

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
        // run it once immediately
        callback.run();
        Notifier notifier = new Notifier(callback);
        // update at 50Hz until robot code takes over
        notifier.startPeriodic(0.02);
        return notifier;
    }

    @Override
    public void periodicAfterCommands() {
        boolean somethingIsReallyWrong =
                aprilTagVision.anyCamerasDisconnected()
                        || gamePieceVision.anyCamerasDisconnected()
                        || superstructure.hood.isEmergencyStopped();
        boolean lowBattery = operatorDashboard.isBatteryVoltageAlertActive();

        if (somethingIsReallyWrong) {
            LEDPatterns.somethingIsReallyWrong.applyTo(buffer);
            updateMechanismAndLog();
            return;
        }

        if (superintake.intakeRollers.highTemperatureAlert.get() ||
                superstructure.flywheel.highTemperatureAlert.get() ||
                superstructure.hood.highTemperatureAlert.get()) {
            LEDPatterns.hotMotors.applyTo(buffer);
            updateMechanismAndLog();
            return;
        }

        if (DriverStation.isDisabled()) {
            LEDPatterns.autoReady.applyTo(buffer);
            updateMechanismAndLog();
            return;
        }

        // Enabled: determine patterns for superintake and superstructure
        LEDPattern superintakePattern = switch (superintake.getGoal()) {
            case IDLE -> null;
            case INTAKE, SHOOT -> LEDPatterns.intaking;
            case EJECT -> LEDPatterns.eject;
            case HOME_INTAKE_PIVOT -> LEDPatterns.homing;
        };

        LEDPattern superstructurePattern = switch (superstructure.getGoal()) {
            case IDLE -> null;
            case SHOOT -> shootingKinematics.isShootingParametersMet()
                    ? LEDPatterns.shooting
                    : (shootingKinematics.isShiftMet() ? LEDPatterns.aiming : LEDPatterns.waitingForShift);
            case SHOOT_FORCE -> LEDPatterns.shootingForced;
            case EJECT -> LEDPatterns.eject;
            case HOME_HOOD -> LEDPatterns.homing;
        };

        int activeCount = 0;
        if (superintakePattern != null) activeCount++;
        if (superstructurePattern != null) activeCount++;

        var shiftInfo = HubShiftTracker.get().getShiftInfo();


        if (!lowBattery) {
            if (activeCount >= 2) {
                (superintakePattern != null ? superintakePattern : LEDPatterns.idle).applyTo(firstQuarterView);
                (superstructurePattern != null ? superstructurePattern : LEDPatterns.idle).applyTo(secondQuarterView);
            } else if (activeCount == 1) {
                if (superintakePattern != null) {
                    superintakePattern.applyTo(leftHalfView);
                } else {
                    (superstructurePattern != null ? superstructurePattern : LEDPatterns.idle).applyTo(leftHalfView);
                }
            } else {
                LEDPatterns.idle.applyTo(leftHalfView);
            }
        } else {
            LEDPatterns.lowBattery.applyTo(leftHalfView);
        }

        double remaining = shiftInfo.remainingTime();
        LEDPattern hubPattern = shiftInfo.active() ? LEDPatterns.hubActive : LEDPatterns.HubInactive;

        boolean blink = remaining <= HUB_BLINK_START_SECONDS;
        if (blink) {
            double t = MathUtil.clamp(remaining / HUB_BLINK_START_SECONDS, 0.0, 1.0);
            double period = HUB_BLINK_MIN_PERIOD + t * (HUB_BLINK_MAX_PERIOD - HUB_BLINK_MIN_PERIOD);
            double phase = Timer.getFPGATimestamp() % period;
            boolean on = phase < (period * 0.5);
            if (on) {
                hubPattern.applyTo(rightHalfView);
            } else {
                LEDPatterns.idle.applyTo(rightHalfView);
            }
        } else {
            hubPattern.applyTo(rightHalfView);
        }

        updateMechanismAndLog();
    }

    // Extracted to keep periodic concise
    private void updateMechanismAndLog() {
        io.setData(buffer);
        for (int i = 0; i < buffer.getLength(); i++) {
            ligaments[i].setColor(new Color8Bit(
                    buffer.getRed(i),
                    buffer.getGreen(i),
                    buffer.getBlue(i)
            ));
        }
        Logger.recordOutput("LEDs/Mechanism", mechanism);
    }
}
