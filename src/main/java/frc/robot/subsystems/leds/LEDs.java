package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.subsystem.Periodic;
import frc.robot.HubShiftTracker;
import frc.robot.OperatorDashboard;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SparkCANcoderHelper;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static frc.robot.subsystems.leds.LEDConstants.createIO;
import static frc.robot.subsystems.leds.LEDConstants.length;

public class LEDs implements Periodic {
    private final LEDsIO io = createIO();
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
    private final AddressableLEDBufferView firstHalfView = new AddressableLEDBufferView(buffer, 0, length / 2 - 1);
    private final AddressableLEDBufferView secondHalfView = new AddressableLEDBufferView(buffer, length / 2, length - 1);

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.5, 2.1, new Color8Bit(Color.kBlack));
    private final LoggedMechanismLigament2d[] ligaments = new LoggedMechanismLigament2d[length];

    @Getter
    private static final LEDs instance = new LEDs();

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

    @Override
    public void periodicAfterCommands() {
        if (DriverStation.isDisabled()) {
            getDisabledPattern().applyTo(buffer);
        } else {
            getEnabledPatternFirstHalf().applyTo(firstHalfView);
            getEnabledPatternSecondHalf().applyTo(secondHalfView);
        }

        io.setData(buffer);
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

    private LEDPattern getDisabledPattern() {
        if (AprilTagVision.getInstance().anyCamerasDisconnected() ||
                //gamePieceVision.anyCamerasDisconnected() ||
                Superstructure.getInstance().hood.isEmergencyStopped() ||
                Superintake.getInstance().intakePivot.isEmergencyStopped() ||
                Superintake.getInstance().isAnythingDisconnected() ||
                Superstructure.getInstance().isAnythingDisconnected() ||
                Drive.getInstance().isAnythingDisconnected() ||
                SparkCANcoderHelper.isAnyResetFailed()) {
            return LEDPatterns.somethingIsReallyWrong;
        }

        if (OperatorDashboard.getInstance().hoodNotHomedAlert.get()) {
            return LEDPatterns.hoodNotHomed;
        }

        if (OperatorDashboard.getInstance().intakePivotNotHomedAlert.get()) {
            return LEDPatterns.intakePivotNotHomed;
        }

        if (OperatorDashboard.getInstance().autoNotChosenAlert.get()) {
            return LEDPatterns.autoNotChosen;
        }

        if (OperatorDashboard.getInstance().isBatteryVoltageAlertActive()) {
            return LEDPatterns.lowBattery;
        }

        //if (operatorDashboard.autoChosen.get() && autoManager.getSelectedAutoStartingPose().isPresent() && !autoManager.isAtAutoStartingPose()) {
        //    LEDPatterns.autoPlacementProgress(autoManager::getPlacementProgress).applyTo(buffer);
        //} else {
        return LEDPatterns.autoReady;
        //}
    }

    private LEDPattern getEnabledPatternFirstHalf() {
        if (AprilTagVision.getInstance().anyCamerasDisconnected() ||
                //gamePieceVision.anyCamerasDisconnected() ||
                //superstructure.hood.isEmergencyStopped() ||
                //superintake.intakePivot.isEmergencyStopped() ||
                HubShiftTracker.getInstance().gameDataBrokenAlert.get() ||
                Superintake.getInstance().isAnythingDisconnected() ||
                Superstructure.getInstance().isAnythingDisconnected() ||
                Drive.getInstance().isAnythingDisconnected() ||
                SparkCANcoderHelper.isAnyResetFailed()) {
            return LEDPatterns.somethingIsReallyWrong;
        }

        //if (
        //        superintake.intakeRollers.highTemperatureAlert.get() ||
        //                superstructure.flywheel.highTemperatureAlert.get() ||
        //                superstructure.hood.highTemperatureAlert.get()
        //) {
        //    return LEDPatterns.hotMotors;
        //}

        if (OperatorDashboard.getInstance().isBatteryVoltageAlertActive()) {
            return LEDPatterns.lowBattery;
        }

        return LEDPatterns.idle;
    }

    private LEDPattern getEnabledPatternSecondHalf() {
        return switch (Superstructure.getInstance().getGoal()) {
            case SHOOT -> ShootingKinematics.getInstance().isShootingParametersMet()
                    ? LEDPatterns.shooting
                    : (
                    ShootingKinematics.getInstance().isShiftMet()
                            ? LEDPatterns.aiming
                            : LEDPatterns.waitingForShift
            );
            case SHOOT_FORCE -> LEDPatterns.shootingForced;
            default -> LEDPatterns.idle;
        };
    }
}

