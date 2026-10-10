package frc.lib.devices.motor;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
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

        mechanismSim = mechanismSimBuilder.build(motor, initialPositionRad, config.getGearRatio());
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

    @Override
    public void setVelocityRequest(double setpointRadPerSec, double arbitraryFeedforwardVolts) {
        // REVLib doesn't include kV when simulating. This is a bug with REVLib.
        // To work around the bug, just calculate the kV term ourselves.
        // See also https://github.com/REVrobotics/REV-Software-Binaries/issues/29
        double ffkV = setpointRadPerSec * spark.configAccessor.closedLoop.feedForward.getkV(ClosedLoopSlot.kSlot1);

        super.setVelocityRequest(setpointRadPerSec, arbitraryFeedforwardVolts + ffkV);
    }
}
