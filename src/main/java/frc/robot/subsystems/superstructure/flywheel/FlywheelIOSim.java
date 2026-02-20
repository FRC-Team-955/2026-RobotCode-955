package frc.robot.subsystems.superstructure.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.PIDF;
import frc.lib.motor.MotorIOSim;
import frc.lib.motor.RequestType;

public class FlywheelIOSim extends FlywheelIO {
    private final MotorIOSim leader;
    private final MotorIOSim follower;

    public FlywheelIOSim(double JKgMetersSquared, DCMotor motor) {
        leader = new MotorIOSim(
                FlywheelConstants.gearRatio,
                JKgMetersSquared,
                motor,
                PIDF.zero(),
                PIDF.zero()
        );

        follower = new MotorIOSim(
                FlywheelConstants.gearRatio,
                JKgMetersSquared,
                motor,
                PIDF.zero(),
                PIDF.zero()
        );
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        leader.updateInputs(inputs.leader);
        follower.setRequest(RequestType.VoltageVolts, inputs.leader.appliedVolts);
        follower.updateInputs(inputs.follower);
    }

    @Override
    public void setVelocityPIDF(PIDF newGains) {
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
