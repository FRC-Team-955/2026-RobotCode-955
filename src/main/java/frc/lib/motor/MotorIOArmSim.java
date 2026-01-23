package frc.lib.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.PIDF;
import frc.lib.Util;

public class MotorIOArmSim extends MotorIO {
    private static final double voltageLimit = 12.0;
    private static final double currentLimitAmps = 40.0;
    private final DCMotor motor;

    private final SingleJointedArmSim armSim;
    private PIDController pid;
    private ArmFeedforward ff;

    private double appliedVolts;
    private boolean closedLoop = true;

    // If using physical values, JKgMetersSquared is the moment of inertia J of the flywheel
    public MotorIOArmSim(DCMotor motor, double gearRatio, double jKgMetersSquared, double armLength, double minAngleRads, double maxAngleRads, boolean simulateGravity, double startingAngleRads, double measurementStdDevs, PIDF gains) {
        this.motor = motor.withReduction(gearRatio);
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
            appliedVolts = MathUtil.clamp(appliedVolts, -voltageLimit, voltageLimit);

            // https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/27ef554d86a62d8dba1a361cc5eca8919d4be1f9/project/src/main/java/org/ironmaple/simulation/motorsims/SimulatedMotorController.java#L63-L84
            final double kCurrentThreshold = 1.2;

            final double motorCurrentVelocityRadPerSec = armSim.getVelocityRadPerSec();
            final double requestedOutputVoltageVolts = appliedVolts;
            final double currentAtRequestedVoltageAmps = motor.getCurrent(motorCurrentVelocityRadPerSec, requestedOutputVoltageVolts);

            // Resource for current limiting:
            // https://file.tavsys.net/control/controls-engineering-in-frc.pdf (sec 12.1.3)
            double limitedVoltage = requestedOutputVoltageVolts;
            final boolean currentTooHigh =
                    Math.abs(currentAtRequestedVoltageAmps) > (kCurrentThreshold * currentLimitAmps);
            if (currentTooHigh) {
                final double limitedCurrent = Math.copySign(currentLimitAmps, currentAtRequestedVoltageAmps);
                limitedVoltage = motor.getVoltage(motor.getTorque(limitedCurrent), motorCurrentVelocityRadPerSec);
            }

            // ensure the current limit doesn't cause an increase to output voltage
            if (Math.abs(limitedVoltage) > Math.abs(requestedOutputVoltageVolts))
                limitedVoltage = requestedOutputVoltageVolts;

            appliedVolts = limitedVoltage;
        }

        armSim.setInputVoltage(appliedVolts);

        armSim.update(0.02);

        inputs.connected = true;
        inputs.positionRad = armSim.getAngleRads();
        inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(armSim.getCurrentDrawAmps());
    }

    @Override
    public void setPositionPIDF(PIDF newGains) {
        System.out.println("Setting motor position gains");
        pid = newGains.toPID();
        ff = newGains.toArmFF();
    }

    @Override
    public void setVelocityPIDF(PIDF newGains) {
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