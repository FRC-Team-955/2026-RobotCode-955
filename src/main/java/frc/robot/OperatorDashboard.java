package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Util;
import frc.lib.network.LoggedNetworkBooleanExt;
import frc.lib.network.LoggedNetworkNumberExt;
import frc.lib.subsystem.Periodic;
import frc.robot.subsystems.drive.DriveConstants;
import lombok.Setter;

import java.util.EnumMap;
import java.util.function.Consumer;

public class OperatorDashboard implements Periodic {
    private final RobotState robotState = RobotState.get();
    private final Controller controller = Controller.get();

    private static final String prefix = "/OperatorDashboard/";

    public final LoggedNetworkBooleanExt coastOverride = new LoggedNetworkBooleanExt(prefix + "CoastOverride", false);
    public final LoggedNetworkBooleanExt coralStuckInRobotMode = new LoggedNetworkBooleanExt(prefix + "CoralStuckInRobotMode", false);
    public final LoggedNetworkBooleanExt manualScoring = new LoggedNetworkBooleanExt(prefix + "ManualScoring", false);
    public final LoggedNetworkBooleanExt ignoreEndEffectorBeamBreak = new LoggedNetworkBooleanExt(prefix + "IgnoreEndEffectorBeamBreak", false);
    public final LoggedNetworkBooleanExt autoChosen = new LoggedNetworkBooleanExt(prefix + "AutoChosen", false);
    public final LoggedNetworkBooleanExt manualReefSide = new LoggedNetworkBooleanExt(prefix + "ManualReefSide", false);
    public final LoggedNetworkBooleanExt forceGamePieceLEDs = new LoggedNetworkBooleanExt(prefix + "ForceGamePieceLEDs", false);

    public final LoggedNetworkBooleanExt elevatorEStop = new LoggedNetworkBooleanExt(prefix + "ElevatorEStop", false);
    public final LoggedNetworkBooleanExt useRealElevatorState = new LoggedNetworkBooleanExt(prefix + "UseRealElevatorState", false);
    public final LoggedNetworkBooleanExt zeroElevator = new LoggedNetworkBooleanExt(prefix + "ZeroElevator", false);
    public final LoggedNetworkNumberExt elevatorOffsetMeters = new LoggedNetworkNumberExt(prefix + "ElevatorOffsetMeters", 0);
    // TODO
//    public final LoggedNetworkBooleanExt manualElevator = new LoggedNetworkBooleanExt(prefix + "ManualElevator", false);
//    public final LoggedNetworkBooleanExt manualElevatorUp = new LoggedNetworkBooleanExt(prefix + "ManualElevatorUp", false);
//    public final LoggedNetworkBooleanExt manualElevatorDown = new LoggedNetworkBooleanExt(prefix + "ManualElevatorDown", false);

    private final Alert coastOverrideAlert = new Alert("Coast override is enabled.", Alert.AlertType.kWarning);
    private final Alert autoNotChosenAlert = new Alert("Auto is not chosen!", Alert.AlertType.kError);
    @SuppressWarnings("FieldCanBeLocal")
    private final Alert constantSetAlert = new Alert("Constants are set.", Alert.AlertType.kInfo);

    public final OperatorKeypad operatorKeypad = new OperatorKeypad();
    private final Alert operatorKeypadDisconnectedAlert = new Alert("Operator keypad is not connected!", Alert.AlertType.kError);

    @Setter
    private boolean ignoreClosestReefSideChanges = false;

    private static OperatorDashboard instance;

    public static OperatorDashboard get() {
        if (instance == null)
            synchronized (OperatorDashboard.class) {
                instance = new OperatorDashboard();
            }

        return instance;
    }

    private OperatorDashboard() {
        if (Constants.tuningMode || DriveConstants.disableDriving || DriveConstants.disableGyro) {
            constantSetAlert.set(true);
        }
    }

    @Override
    public void periodicBeforeCommands() {
        if (operatorKeypad.isConnected()) {
            operatorKeypadDisconnectedAlert.set(false);
            operatorKeypad.update();

            if (operatorKeypad.canUseManualReefZoneSide) {
            } else if (operatorKeypad.canUseOverrides) {
            }

            if (operatorKeypad.canUseOverrides) {
            }
        } else {
            operatorKeypadDisconnectedAlert.set(true);
        }

        // Note - we only handle alerts for general overrides.
        // So subsystem toggles are handled in their respective subsystems
        coastOverrideAlert.set(coastOverride.get());
        autoNotChosenAlert.set(!autoChosen.get());
    }

    private static <E extends Enum<E>> void updateToggles(
            EnumMap<E, LoggedNetworkBooleanExt> map,
            E currentlySelected
    ) {
        LoggedNetworkBooleanExt toggle = map.get(currentlySelected);
        // Set the corresponding toggle to true
        toggle.set(true);
        // Set the rest to false
        for (var entry1 : map.entrySet()) {
            if (entry1.getKey() != currentlySelected) {
                entry1.getValue().set(false);
            }
        }
    }

    private static <E extends Enum<E>> void handleEnumToggles(
            EnumMap<E, LoggedNetworkBooleanExt> map,
            E currentlySelected,
            Consumer<E> select
    ) {
        // If none are toggled
        if (map.values().stream().noneMatch(LoggedNetworkBooleanExt::get)) {
            // Enable the last selected one
            map.get(currentlySelected).set(true);
        } else {
            // Otherwise, look for changes in the toggles
            for (var entry : map.entrySet()) {
                LoggedNetworkBooleanExt toggle = entry.getValue();
                // If it's toggled
                if (toggle.get()) {
                    E key = entry.getKey();
                    // If it wasn't already selected
                    if (key != currentlySelected) {
                        // Select the new value
                        select.accept(key);
                        // Set the rest to false
                        for (var entry1 : map.entrySet()) {
                            if (entry1.getKey() != key) {
                                entry1.getValue().set(false);
                            }
                        }
                        break;
                    }
                }
            }
        }
    }

    private static <E extends Enum<E>> EnumMap<E, LoggedNetworkBooleanExt> generateTogglesForEnum(String name, E[] enumValues, Class<E> enumClass) {
        return Util.createEnumMap(enumClass, enumValues, (side) -> new LoggedNetworkBooleanExt(prefix + name + "/" + side.name(), false));
    }

    public static class OperatorKeypad {
        private final GenericHID hid = new GenericHID(1);

        private boolean canUseManualReefZoneSide = false;
        private boolean canUseOverrides = true;

        private boolean lastManualReefSide = false;
        private final Timer sinceManualReefSideChanged = new Timer();

        private void update() {
            // It takes a bit of time for the buttons to switch whenever
            // use manual reef side is changed, since we have to use the
            // same buttons for reef side and overrides. To ensure that
            // we don't process something unwanted (such as enabling an
            // override we do not want), we don't want to allow either
            // reef side or overrides to be read for 0.5 seconds after
            // use manual reef side is changed.
            boolean manualReefSide = getManualReefSide();
            if (lastManualReefSide != manualReefSide) {
                // We have a change and haven't dealt with it, start the timer
                sinceManualReefSideChanged.restart();
                // Don't allow anything to change while waiting
                canUseManualReefZoneSide = false;
                canUseOverrides = false;
                // If manual reef side is switched while we are waiting,
                // we want to restart the timer again
                lastManualReefSide = manualReefSide;
            }
            if (sinceManualReefSideChanged.isRunning() && sinceManualReefSideChanged.hasElapsed(0.2)) {
                // If it's been long enough, stop the timer and allow
                // either overrides or manual reef zone side to be used
                sinceManualReefSideChanged.stop();
                // At this point, both of them should be false, so we only need
                // to change one of them to true
                if (manualReefSide) {
                    canUseManualReefZoneSide = true;
                } else {
                    canUseOverrides = true;
                }
            }
        }

        private boolean isConnected() {
            return hid.isConnected();
        }

        private boolean getOverride1() {
            // Override 1 is a toggle
            return hid.getRawButton(1);
        }

        private boolean getOverride2() {
            // Override 2 is a toggle
            return hid.getRawButton(2);
        }

        private boolean getOverride3() {
            // Override 3 is a toggle
            return hid.getRawButton(3);
        }

        public Trigger getOverride4() {
            // Override 4 is a simple button; will be disabled upon release
            return new Trigger(() -> canUseOverrides && hid.getRawButton(4));
        }

        private boolean getOverride5() {
            // Override 5 is a toggle
            return hid.getRawButton(5);
        }

        public Trigger getOverride6() {
            // Override 6 is a simple button; will be disabled upon release
            return new Trigger(() -> canUseOverrides && hid.getRawButton(6));
        }

        private boolean getManualReefSide() {
            return hid.getRawButton(13);
        }
    }
}
