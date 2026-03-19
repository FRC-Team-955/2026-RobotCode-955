package frc.lib.device;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import lombok.AccessLevel;
import lombok.RequiredArgsConstructor;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

@SuppressWarnings("ClassCanBeRecord")
@RequiredArgsConstructor(access = AccessLevel.PRIVATE)
public class MechanismSim {
    private final double gearRatio;
    private final DoubleConsumer iterate;
    private final DoubleSupplier mechanismPositionRad;
    private final DoubleSupplier mechanismVelocityRadPerSec;

    public void update(TalonFXSimState talonSim) {
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/simulation/simulation-intro.html

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        iterate.accept(talonSim.getMotorVoltage());

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonSim.setRawRotorPosition(mechanismPositionRad.getAsDouble() * gearRatio);
        talonSim.setRotorVelocity(mechanismVelocityRadPerSec.getAsDouble() * gearRatio);
    }

    public void update(SparkMaxSim sparkSim) {
        // https://docs.revrobotics.com/revlib/spark/sim/simulation-getting-started

        // update the simulation of what our mechanism is doing
        final double batteryVoltage = 12.0;
        iterate.accept(sparkSim.getAppliedOutput() * batteryVoltage);

        // update the spark max
        sparkSim.iterate(
                mechanismVelocityRadPerSec.getAsDouble(),
                batteryVoltage,
                Constants.loopPeriod
        );
    }

    @FunctionalInterface
    public interface Builder {
        MechanismSim build(DCMotor motor, double initialPositionRad);
    }

    public static Builder roller(
            double gearRatio,
            double JKgMetersSquared
    ) {
        return (motor, initialPositionRad) -> {
            DCMotorSim motorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(motor, JKgMetersSquared, gearRatio),
                    motor,
                    0.004,
                    0.0
            );
            return new MechanismSim(
                    gearRatio,
                    (inputVoltage) -> {
                        motorSim.setInputVoltage(inputVoltage);
                        motorSim.update(Constants.loopPeriod);
                    },
                    motorSim::getAngularPositionRad,
                    motorSim::getAngularVelocityRadPerSec
            );
        };
    }

    public static Builder arm(
            double gearRatio,
            double JKgMetersSquared,
            double armLength,
            double minAngleRads,
            double maxAngleRads,
            boolean simulateGravity
    ) {
        return (motor, initialPositionRad) -> {
            SingleJointedArmSim armSim = new SingleJointedArmSim(
                    motor,
                    gearRatio,
                    JKgMetersSquared,
                    armLength,
                    minAngleRads,
                    maxAngleRads,
                    simulateGravity,
                    initialPositionRad,
                    0.004,
                    0.0
            );
            return new MechanismSim(
                    gearRatio,
                    (inputVoltage) -> {
                        armSim.setInputVoltage(inputVoltage);
                        armSim.update(Constants.loopPeriod);
                    },
                    armSim::getAngleRads,
                    armSim::getVelocityRadPerSec
            );
        };
    }
}
