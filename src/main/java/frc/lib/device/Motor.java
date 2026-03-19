package frc.lib.device;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import frc.lib.network.LoggedTunablePIDF;
import org.jetbrains.annotations.Nullable;

public class Motor extends Device<MotorIO, MotorIOInputsAutoLogged> {
    private final @Nullable LoggedTunablePIDF gains;

    /** Added to raw position to get wanted position. See getPositionRad, setPositionRequest, and setEncoderPosition for more info. */
    private double positionOffsetRad = 0.0;

    private boolean emergencyStopped = false;

    private final Alert highTemperatureAlert;
    private final Alert emergencyStoppedAlert;

    public Motor(String name, MotorIO.Builder ioBuilder) {
        this(name, null, ioBuilder);
    }

    public Motor(String name, @Nullable LoggedTunablePIDF gains, MotorIO.Builder ioBuilder) {
        super(name, ioBuilder.build(gains), new MotorIOInputsAutoLogged());

        this.gains = gains;

        highTemperatureAlert = new Alert(name + " temperature is high.", Alert.AlertType.kWarning);
        emergencyStoppedAlert = new Alert(name + " is emergency stopped.", Alert.AlertType.kError);
    }

    @Override
    void updateAndProcessInputs() {
        super.updateAndProcessInputs();

        highTemperatureAlert.set(getTemperatureCelsius() > 50.0);

        if (gains != null && gains.hasChanged()) {
            System.out.println("Setting gains of " + name);
            io.setGains(gains);
        }
    }

    @Override
    public boolean isConnected() {
        return inputs.connected;
    }

    public double getPositionRad() {
        // raw + offset = wanted
        return inputs.positionRad + positionOffsetRad;
    }

    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }

    public double getAppliedVolts() {
        return inputs.appliedVolts;
    }

    public double getStatorCurrentAmps() {
        return inputs.statorCurrentAmps;
    }

    public double getSupplyCurrentAmps() {
        return inputs.supplyCurrentAmps;
    }

    public double getTemperatureCelsius() {
        return inputs.temperatureCelsius;
    }

    public void setEmergencyStopped(boolean emergencyStopped, NeutralModeValue neutralModeIfEmergencyStopUndone) {
        if (emergencyStopped != this.emergencyStopped) {
            if (emergencyStopped) {
                System.out.println("Emergency stopping " + name);
                setVoltageRequest(0.0);
                setNeutralMode(NeutralModeValue.Coast);
            } else {
                System.out.println("Undoing emergency stop for " + name);
                setNeutralMode(neutralModeIfEmergencyStopUndone);
            }
            this.emergencyStopped = emergencyStopped;
            emergencyStoppedAlert.set(emergencyStopped);
        }
    }

    public void setVoltageRequest(double volts) {
        if (!emergencyStopped) {
            io.setVoltageRequest(volts);
        }
    }

    public void setPositionRequest(double setpointRad) {
        if (!emergencyStopped) {
            // raw = wanted - offset
            io.setPositionRequest(setpointRad - positionOffsetRad);
        }
    }

    public void setVelocityRequest(double setpointRadPerSec) {
        if (!emergencyStopped) {
            io.setVelocityRequest(setpointRadPerSec);
        }
    }

    /**
     * NOTE: BLOCKS THE MAIN THREAD!!! ONLY CALL ON STARTUP!!!!
     * <p>
     * /**
     * Intended to be used like this:
     * <pre>
     *     private final Motor motor = new Motor(
     *         ...
     *     ).withFollowRequest(...);
     * </pre>
     */
    public Motor withFollowRequest(Motor leader, MotorAlignmentValue alignment) {
        System.out.println("Making " + name + " follow " + leader.name);
        io.setFollowRequest(leader.io, alignment);
        return this;
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        System.out.println("Setting " + name + " neutral mode to " + neutralMode);
        io.setNeutralMode(neutralMode);
    }

    public void setEncoderPosition(double positionRad) {
        System.out.println("Setting " + name + " encoder position to " + positionRad);
        // offset = wanted - raw
        positionOffsetRad = positionRad - inputs.positionRad;
    }

    /**
     * Intended to be used like this:
     * <pre>
     *     private final Motor motor = new Motor(
     *         ...
     *     ).withInitialEncoderPosition(...);
     * </pre>
     */
    public Motor withInitialEncoderPosition(double positionRad) {
        setEncoderPosition(positionRad);
        return this;
    }
}
