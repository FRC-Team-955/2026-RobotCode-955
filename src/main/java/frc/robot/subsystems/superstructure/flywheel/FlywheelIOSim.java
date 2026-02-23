package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

public class FlywheelIOSim extends FlywheelIO {
    private final MotorIOSim leader;
    private final MotorIOSim follower;

    public FlywheelIOSim(double JKgMetersSquared, DCMotor motor) {
        leader = new MotorIOSim(
                FlywheelConstants.gearRatio,
                JKgMetersSquared,
                motor,
                null,
                null,
                0.0
        );

        follower = new MotorIOSim(
                FlywheelConstants.gearRatio,
                JKgMetersSquared,
                motor,
                null,
                null,
                0.0
        );
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        leader.updateInputs(inputs.leader);
        follower.setRequest(RequestType.VoltageVolts, inputs.leader.appliedVolts);
        follower.updateInputs(inputs.follower);
    }

    @Override
    public void setVelocityPIDF(LoggedTunablePIDF newGains) {
        leader.setVelocityPIDF(newGains);
    }

    @Override
    public void setVelocityRequest(double setpointRadPerSec) {
        leader.setRequest(RequestType.VelocityRadPerSec, setpointRadPerSec);
    }

    @Override
    public void setStopRequest() {
        leader.setRequest(RequestType.VoltageVolts, 0);
    }
}
