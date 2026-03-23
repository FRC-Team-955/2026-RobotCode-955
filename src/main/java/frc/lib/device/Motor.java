package frc.lib.device;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.network.LoggedTunablePIDF;

public class Motor extends Device<MotorIO, MotorIOInputsAutoLogged> {
    //private final @Nullable LoggedTunablePIDF gains;

    private boolean emergencyStopped = false;

    private final Alert highTemperatureAlert;
    private final Alert emergencyStoppedAlert;

    public Motor(String name, MotorIO io) {
        super(name, io, new MotorIOInputsAutoLogged());

        highTemperatureAlert = new Alert(name + " temperature is high.", Alert.AlertType.kWarning);
        emergencyStoppedAlert = new Alert(name + " is emergency stopped.", Alert.AlertType.kError);
    }

    //public Motor(String name MotorIO io) {
    //
    //
    //    //this.gains = gains;
    //
    //
    //}


    @Override
    void updateAndProcessInputs() {
        super.updateAndProcessInputs();

        highTemperatureAlert.set(getTemperatureCelsius() > 50.0);

        //if (gains != null && gains.hasChanged()) {
        //    System.out.println("Setting gains of " + name);
        //    io.setGains(gains);
        //}
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
            io.setPositionRequest(setpointRad);
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
     * NOTE 2: Emergency stopping will break following. More thought needs to be
     * put into this for it to work. TODO: maybe just rewrite the whole MotorIO to be
     * more follower friendly idk
     */
    public void setFollowRequest(Motor leader, MotorAlignmentValue alignment) {
        System.out.println("Making " + name + " follow " + leader.name);
        io.setFollowRequest(leader.io, alignment);
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
