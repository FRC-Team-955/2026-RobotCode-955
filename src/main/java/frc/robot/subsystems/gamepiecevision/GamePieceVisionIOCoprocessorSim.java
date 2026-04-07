package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.SimManager;
import frc955.gamepiecevision.SharedGamePieceVisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class GamePieceVisionIOCoprocessorSim extends GamePieceVisionIOCoprocessor {
    private static final SimManager simManager = SimManager.get();

    protected final PhotonCamera camera;

    public GamePieceVisionIOCoprocessorSim(String name) {
        camera = new PhotonCamera(name);

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(640, 1000, Rotation2d.fromDegrees(150.0));
        cameraProperties.setCalibError(0, 0);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(45);
        cameraProperties.setLatencyStdDevMs(10);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProperties);
        cameraSim.setMaxSightRange(3.0);
        // Disable rendering of camera streams for debugging/dashboard display
        cameraSim.enableRawStream(false);
        cameraSim.enableProcessedStream(false);

        simManager.gamePieceVisionSystem.addCamera(cameraSim, SharedGamePieceVisionConstants.robotToCamera);
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {
        simManager.ensureGamePieceVisionSystemUpdated();
        super.updateInputs(inputs);
    }
}
