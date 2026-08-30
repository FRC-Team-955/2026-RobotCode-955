package frc.lib.devices.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import frc.lib.devices.device.Device;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;

public class Motor extends Device<MotorIO, MotorIOInputsAutoLogged> {
    private final Alert highTemperatureAlert;
    private final Alert emergencyStoppedAlert;

    public Motor(String name, MotorIO io) {
        super(name, io, new MotorIOInputsAutoLogged());

        highTemperatureAlert = new Alert(name + " temperature is high.", Alert.AlertType.kWarning);
        emergencyStoppedAlert = new Alert(name + " is emergency stopped.", Alert.AlertType.kError);
    }

    public static Motor createSparkMax(String name, int canID, CtrlSparkMaxConfig config, double initialPositionRad, MechanismSim.Builder mechanismSimBuilder) {
        return new Motor(name, switch (BuildConstants.mode) {
            case REAL -> new MotorIOSparkMax(canID, config, initialPositionRad);
            case SIM -> new MotorIOSparkMaxSim(config, initialPositionRad, mechanismSimBuilder);
            case REPLAY -> new MotorIOReplay();
        });
    }

    /** For common config options, see the javadocs for {@link MotorIOTalonFX#MotorIOTalonFX(int, TalonFXConfiguration, double)} */
    public static Motor createTalonFX(String name, int canID, TalonFXConfiguration config, double initialPositionRad, MechanismSim.Builder mechanismSimBuilder) {
        return new Motor(name, switch (BuildConstants.mode) {
            case REAL -> new MotorIOTalonFX(canID, config, initialPositionRad);
            case SIM -> new MotorIOTalonFXSim(config, initialPositionRad, mechanismSimBuilder);
            case REPLAY -> new MotorIOReplay();
        });
    }

    @Override
    protected void updateAndProcessInputs() {
        super.updateAndProcessInputs();

        highTemperatureAlert.set(getTemperatureCelsius() > 50.0);
    }

    @Override
    public boolean isConnected() {
        return inputs.connected;
    }

    public double getPositionRad() {
        return inputs.positionRad;
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

    private boolean emergencyStopped = false;

    public void setEmergencyStopped(boolean emergencyStopped, NeutralModeValue normalNeutralMode) {
        if (emergencyStopped != this.emergencyStopped) {
            if (emergencyStopped) {
                System.out.println("Emergency stopping " + name);
                setVoltageRequest(0.0);
                setNeutralMode(NeutralModeValue.Coast);
            } else {
                System.out.println("Undoing emergency stop for " + name);
                setNeutralMode(normalNeutralMode);
                reinstateFollower.run();
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
            io.setPositionRequest(setpointRad);
        }
    }

    public void setVelocityRequest(double setpointRadPerSec) {
        if (!emergencyStopped) {
            io.setVelocityRequest(setpointRadPerSec);
        }
    }

    /** Used to re-enable following after emergency stop is disabled. */
    private Runnable reinstateFollower = () -> {};

    /**
     * NOTE: BLOCKS THE MAIN THREAD!!! ONLY CALL ON STARTUP!!!!
     */
    public void setFollowRequest(Motor leader, MotorAlignmentValue alignment) {
        reinstateFollower = () -> {
            System.out.println("Making " + name + " follow " + leader.name);
            io.setFollowRequest(leader.io, alignment);
        };
        reinstateFollower.run();
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        System.out.println("Setting " + name + " neutral mode to " + neutralMode);
        io.setNeutralMode(neutralMode);
    }

    public void setGains(LoggedTunablePIDF newGains) {
        System.out.println("Setting " + name + " gains to " + newGains);
        io.setGains(newGains);
    }

    /**
     * NOTE: The position will not instantly change!! Keep this in mind!
     * You may want to add a delay before returning to closed loop control
     * so that the motor does not attempt to move to an invalid position
     * <p>
     * TODO: maybe just add a Timer in the Motor class so this happens for all motors;
     * only allow setting position request if a certain delay has been reached
     */
    public void setEncoderPosition(double positionRad) {
        System.out.println("Setting " + name + " encoder position to " + positionRad);
        io.setEncoderPosition(positionRad);
    }
}
