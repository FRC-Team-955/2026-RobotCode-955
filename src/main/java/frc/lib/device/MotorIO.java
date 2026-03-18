package frc.lib.device;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.network.LoggedTunablePIDF;
import org.littletonrobotics.junction.AutoLog;

public abstract class MotorIO extends DeviceIO<MotorIOInputsAutoLogged> {
    @AutoLog
    public static class MotorIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public abstract void setVoltageRequest(double volts);

    public abstract void setPositionRequest(double setpointRad);

    public abstract void setVelocityRequest(double setpointRadPerSec);

    public abstract void setFollowRequest(MotorIO leaderIO, MotorAlignmentValue alignment);

    public abstract void setGains(LoggedTunablePIDF newGains);

    public abstract void setNeutralMode(NeutralModeValue neutralMode);

    public abstract void setEncoderPosition(double positionRad);
}
