package frc.lib.device;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.network.LoggedTunablePIDF;

public class MotorIOReplay extends MotorIO {
    @Override
    public void updateInputs(MotorIOInputsAutoLogged inputs) {
    }

    @Override
    public void setVoltageRequest(double volts) {
    }

    @Override
    public void setPositionRequest(double setpointRad) {
    }

    @Override
    public void setVelocityRequest(double setpointRadPerSec) {
    }

    @Override
    public void setFollowRequest(MotorIO leaderIO, MotorAlignmentValue alignment) {
    }

    @Override
    public void setGains(LoggedTunablePIDF newGains) {
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
    }

    @Override
    public void setEncoderPosition(double positionRad) {
    }
}
