package frc.lib.device;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.SimManager;

public class MotorIOSparkMaxSim extends MotorIOSparkMax {
    private final SparkMaxSim sparkSim;
    private final MechanismSim mechanismSim;

    public MotorIOSparkMaxSim(CtrlSparkMaxConfig config, double initialPositionRad, MechanismSim.Builder mechanismSimBuilder) {
        super(SimManager.getNewCANId(), config, initialPositionRad);

        // A single neo is usually not correct, but it's good enough for sim
        DCMotor motor = DCMotor.getNEO(1);
        sparkSim = new SparkMaxSim(spark, motor);

        mechanismSim = mechanismSimBuilder.build(motor, initialPositionRad);
    }

    @Override
    public void updateInputs(MotorIOInputsAutoLogged inputs) {
        mechanismSim.update(sparkSim);

        super.updateInputs(inputs);
    }

    @Override
    public void setEncoderPosition(double positionRad) {
        mechanismSim.setMechanismPositionRad.accept(positionRad);

        super.setEncoderPosition(positionRad);
    }
}
