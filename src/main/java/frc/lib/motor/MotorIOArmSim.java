package frc.lib.motor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.network.LoggedTunablePIDF;
import frc.lib.Util;
import frc.robot.Constants;

public class MotorIOArmSim extends MotorIO {
    private final SingleJointedArmSim armSim;
    private PIDController pid;
    private ArmFeedforward ff;

    private double appliedVolts;
    private boolean closedLoop = true;

    // If using physical values, JKgMetersSquared is the moment of inertia J of the flywheel
    public MotorIOArmSim(
            DCMotor motor,
            double gearRatio,
            double jKgMetersSquared,
            double armLength,
            double minAngleRads,
            double maxAngleRads,
            boolean simulateGravity,
            double startingAngleRads,
            double measurementStdDevs,
            LoggedTunablePIDF gains
    ) {
        armSim = new SingleJointedArmSim(
                motor,
                gearRatio,
                jKgMetersSquared,
                armLength,
                minAngleRads,
                maxAngleRads,
                simulateGravity,
                startingAngleRads,
                measurementStdDevs,
                measurementStdDevs
        );

        pid = gains.toPID();
        ff = gains.toArmFF();
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = pid.calculate(armSim.getAngleRads()) + ff.calculate(armSim.getAngleRads(), 0);
        }

        armSim.setInputVoltage(appliedVolts);

        armSim.update(Constants.loopPeriod);

        inputs.connected = true;
        inputs.positionRad = armSim.getAngleRads();
        inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(armSim.getCurrentDrawAmps());
    }

    @Override
    public void setPositionPIDF(LoggedTunablePIDF newGains) {
        System.out.println("Setting motor position gains");
        pid = newGains.toPID();
        ff = newGains.toArmFF();
    }

    @Override
    public void setVelocityPIDF(LoggedTunablePIDF newGains) {
        Util.error("Motor should only set position PIDF");
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting motor brake mode to " + enable);
    }

    @Override
    public void setRequest(RequestType type, double value) {
        switch (type) {
            case PositionRad -> {
                closedLoop = true;
                pid.setSetpoint(value);
            }
            case VoltageVolts -> {
                closedLoop = false;
                appliedVolts = 0.0;
            }
            default -> Util.error("Motor should only use PositionRad");
        }
    }
}