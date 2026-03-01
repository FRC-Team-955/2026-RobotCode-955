package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.SimManager;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class GamePieceVisionIOPhotonVisionSim extends GamePieceVisionIOPhotonVision {
    private static final SimManager simManager = SimManager.get();

    public GamePieceVisionIOPhotonVisionSim(String name, Transform3d robotToCamera) {
        super(name);

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(120.0));
        cameraProperties.setCalibError(0, 0);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(45);
        cameraProperties.setLatencyStdDevMs(10);
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProperties);
        cameraSim.setMaxSightRange(3.0);
        simManager.gamePieceVisionSystem.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {
        simManager.ensureGamePieceVisionSystemUpdated();
        super.updateInputs(inputs);
    }
}
