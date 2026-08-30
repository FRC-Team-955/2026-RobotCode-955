package frc.lib.devices.motor;

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
    public void setPositionRequest(double setpointRad, double arbitraryFeedforwardVolts) {
    }

    @Override
    public void setVelocityRequest(double setpointRadPerSec, double arbitraryFeedforwardVolts) {
    }

    @Override
    public void setFollowRequest(MotorIO leaderIO, MotorAlignmentValue alignment) {
    }

    @Override
    public void setPositionGains(LoggedTunablePIDF newGains) {
    }

    @Override
    public void setVelocityGains(LoggedTunablePIDF newGains) {
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
    }

    @Override
    public void setEncoderPosition(double positionRad) {
    }
}
