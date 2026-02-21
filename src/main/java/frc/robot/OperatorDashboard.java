package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.Util;
import frc.lib.network.LoggedNetworkBooleanExt;
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

    private static final String prefix = "/OperatorDashboard/";

    public final LoggedNetworkBooleanExt coastOverride = new LoggedNetworkBooleanExt(prefix + "CoastOverride", false);
    public final LoggedNetworkBooleanExt autoChosen = new LoggedNetworkBooleanExt(prefix + "AutoChosen", false);
    private final LoggedNetworkBooleanExt fixedHood = new LoggedNetworkBooleanExt(prefix + "Fixed Hood", false);

    @Getter
    private ScoringMode selectedScoringMode = ScoringMode.ShootAndPassAutomatic;
    private final EnumMap<ScoringMode, LoggedNetworkBooleanExt> scoringModeToggles = generateTogglesForEnum("ScoringMode", ScoringMode.values(), ScoringMode.class);

    private final Alert coastOverrideAlert = new Alert("Coast override is enabled.", Alert.AlertType.kWarning);
    private final Alert autoNotChosenAlert = new Alert("Auto is not chosen!", Alert.AlertType.kError);
    @SuppressWarnings("FieldCanBeLocal")
    private final Alert constantSetAlert = new Alert("Constants are set.", Alert.AlertType.kInfo);
    private final Alert batteryVoltageAlert = new Alert("Battery is below 12 Volts!", Alert.AlertType.kError);
    private final Alert fixedHoodAlert = new Alert("Fixed hood mode is enabled.", Alert.AlertType.kWarning);

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
        AutoManager.get().updateAlert();
        batteryVoltageAlert.set(lowBatteryDebouncer.calculate(RobotController.getBatteryVoltage() <= 12.0));
        fixedHoodAlert.set(fixedHood.get());
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
