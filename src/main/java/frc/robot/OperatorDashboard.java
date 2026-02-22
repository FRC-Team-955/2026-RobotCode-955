package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.Util;
import frc.lib.network.LoggedNetworkBooleanExt;
import frc.lib.network.LoggedNetworkNumberExt;
import frc.lib.subsystem.Periodic;
import frc.robot.autos.AutoManager;
import frc.robot.controller.Controller;
import frc.robot.subsystems.drive.DriveConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.EnumMap;
import java.util.function.Consumer;


public class OperatorDashboard implements Periodic {
    public enum ScoringMode {
        ShootAndPassAutomatic,
        ShootHubManual,
        ShootTowerManual,
        PassManual,
    }

    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();
    private static final AutoManager autoManager = AutoManager.get();

    private static final String prefix = "/OperatorDashboard/";

    // Buttons
    public final LoggedNetworkBooleanExt homeIntakePivot = new LoggedNetworkBooleanExt(prefix + "HomeIntakePivot", false);
    public final LoggedNetworkBooleanExt homeHood = new LoggedNetworkBooleanExt(prefix + "HomeHood", false);

    // Toggles and overrides
    public final LoggedNetworkBooleanExt coastOverride = new LoggedNetworkBooleanExt(prefix + "CoastOverride", false);
    public final LoggedNetworkBooleanExt autoChosen = new LoggedNetworkBooleanExt(prefix + "AutoChosen", false);
    public final LoggedNetworkBooleanExt fixedHood = new LoggedNetworkBooleanExt(prefix + "FixedHood", false);
    public final LoggedNetworkBooleanExt manualAiming = new LoggedNetworkBooleanExt(prefix + "ManualAiming", false);
    public final LoggedNetworkBooleanExt disableCANrange = new LoggedNetworkBooleanExt(prefix + "DisableCANrange", false);
    public final LoggedNetworkNumberExt flywheelSmudgeRPM = new LoggedNetworkNumberExt(prefix + "FlywheelSmudgeRPM", 0.0);
    public final LoggedNetworkNumberExt hoodSmudgeDegrees = new LoggedNetworkNumberExt(prefix + "HoodSmudgeDegrees", 0.0);

    @Getter
    private ScoringMode selectedScoringMode = ScoringMode.ShootAndPassAutomatic;
    private final EnumMap<ScoringMode, LoggedNetworkBooleanExt> scoringModeToggles = generateTogglesForEnum("ScoringMode", ScoringMode.values(), ScoringMode.class);

    // Alerts
    public final Alert intakePivotNotHomedAlert = new Alert("Intake pivot has not been homed!", Alert.AlertType.kError);
    public final Alert hoodNotHomedAlert = new Alert("Hood has not been homed!", Alert.AlertType.kError);
    private final Alert coastOverrideAlert = new Alert("Coast override is enabled.", Alert.AlertType.kWarning);
    private final Alert autoNotChosenAlert = new Alert("Auto is not chosen!", Alert.AlertType.kError);

    private final Alert autoNotAlignedAlert = new Alert("Robot is not aligned for auto!", Alert.AlertType.kError);
    @SuppressWarnings("FieldCanBeLocal")
    private final Alert constantSetAlert = new Alert("Constants are set.", Alert.AlertType.kInfo);
    private final Alert batteryVoltageAlert = new Alert("Battery is below 12 volts!", Alert.AlertType.kError);
    private final Alert fixedHoodAlert = new Alert("Fixed hood mode is enabled.", Alert.AlertType.kWarning);
    private final Alert manualAimingAlert = new Alert("Manual aiming is enabled.", Alert.AlertType.kWarning);
    private final Alert disabledCANrangeAlert = new Alert("CANrange is disabled.", Alert.AlertType.kWarning);
    private final Alert smudgesNotZeroAlert = new Alert("Smudges are not zero.", Alert.AlertType.kWarning);

    private final Debouncer lowBatteryDebouncer = new Debouncer(10.0, Debouncer.DebounceType.kRising);

    private static OperatorDashboard instance;

    public static synchronized OperatorDashboard get() {
        if (instance == null) {
            instance = new OperatorDashboard();
        }

        return instance;
    }

    private OperatorDashboard() {
        if (instance != null) {
            Util.error("Duplicate OperatorDashboard created");
        }

        intakePivotNotHomedAlert.set(true);
        hoodNotHomedAlert.set(true);

        if (BuildConstants.tuningMode || DriveConstants.disableDriving || DriveConstants.disableGyro) {
            constantSetAlert.set(true);
        }
    }

    @Override
    public void periodicBeforeCommands() {
        handleEnumToggles(scoringModeToggles, selectedScoringMode, selectNew -> selectedScoringMode = selectNew);
        Logger.recordOutput("OperatorDashboard/SelectedScoringMode", selectedScoringMode);

        // Note - we only handle alerts for general overrides.
        // So subsystem toggles are handled in their respective subsystems
        coastOverrideAlert.set(coastOverride.get());
        autoNotChosenAlert.set(!autoChosen.get());
        autoNotAlignedAlert.set(DriverStation.isDisabled() && !autoManager.isAtAutoStartingPose());
        batteryVoltageAlert.set(lowBatteryDebouncer.calculate(RobotController.getBatteryVoltage() <= 12.0));
        fixedHoodAlert.set(fixedHood.get());
        manualAimingAlert.set(manualAiming.get());
        disabledCANrangeAlert.set(disableCANrange.get());
        smudgesNotZeroAlert.set(flywheelSmudgeRPM.get() != 0 || hoodSmudgeDegrees.get() != 0);
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

    public boolean isBatteryVoltageAlertActive() {
        return batteryVoltageAlert.get();
    }
}
