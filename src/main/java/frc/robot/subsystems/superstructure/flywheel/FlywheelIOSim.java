package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

public class FlywheelIOSim extends FlywheelIO {
    private final MotorIOSim motor;

    public FlywheelIOSim(double JKgMetersSquared, DCMotor motor) {
        this.motor = new MotorIOSim(
                FlywheelConstants.gearRatio,
                JKgMetersSquared,
                motor,
                null,
                FlywheelConstants.velocityGains,
                0.0
        );
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        motor.updateInputs(inputs.leader);
        inputs.follower = inputs.leader;
    }

    @Override
    public void setVelocityPIDF(LoggedTunablePIDF newGains) {
        motor.setVelocityPIDF(newGains);
    }

    @Override
    public void setVelocityRequest(double setpointRadPerSec) {
        motor.setRequest(RequestType.VelocityRadPerSec, setpointRadPerSec);
    }

    @Override
    public void setStopRequest() {
        motor.setRequest(RequestType.VoltageVolts, 0);
    }
}
