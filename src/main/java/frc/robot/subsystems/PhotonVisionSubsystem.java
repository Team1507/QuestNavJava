package frc.robot.subsystems;

import java.util.Optional;

// PhotonVision Libraries
import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

// WPI libraries
import edu.wpi.first.math.geometry.Pose2d;

// Vision base class
import frc.robot.subsystems.vision.VisionSystem;

// Robot Utilities
import frc.robot.utilities.Telemetry;

// Robot Constants
import static frc.robot.Constants.IO.*;
import static frc.robot.Constants.FieldElements.*;

/**
 * VisionSystem implementation for PhotonVision.
 *
 * Responsible for:
 *  - Reading AprilTag detections from PhotonVision
 *  - Running the PhotonPoseEstimator
 *  - Updating latestPose for the VisionSystem base class
 *  - Enforcing multi-tag reliability filtering
 */
public class PhotonVisionSubsystem extends VisionSystem {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    /**
     * Constructor for the PhotonVisionSubsystem.
     *
     * @param drivetrain The drivetrain whose pose may be reset by this system.
     * @param logger     Telemetry logger for publishing vision poses.
     * @param cameraName The name of the camera as configured in PhotonVision UI.
     */
    public PhotonVisionSubsystem(CommandSwerveDrivetrain drivetrain, Telemetry logger, String cameraName) {
        super(drivetrain, logger);

        this.camera = new PhotonCamera(cameraName);

        // MULTI_TAG_PNP_ON_COPROCESSOR = best for accuracy and stability
        this.poseEstimator = new PhotonPoseEstimator(
            APRILTAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            CAMERA_TO_ROBOT
        );
    }

    /**
     * Updates the latest pose from PhotonVision.
     * Called automatically by VisionSystem.periodic().
     */
    @Override
    protected void update() {
        PhotonPipelineResult result = camera.getLatestResult();

        // Require at least 2 tags for reliability
        if (!result.hasTargets() || result.getTargets().size() < 2) {
            latestPose = new Pose2d();
            return;
        }

        Optional<EstimatedRobotPose> estimated = poseEstimator.update(result);
        if (estimated.isEmpty()) {
            latestPose = new Pose2d();
            return;
        }

        latestPose = estimated.get().estimatedPose.toPose2d();

        // If you want to fuse into drivetrain estimator:
        // drivetrain.addVisionMeasurement(latestPose, estimated.get().timestampSeconds, PHOTONVISION_STD_DEVS);
    }

    /**
     * Returns whether PhotonVision currently has valid tracking data.
     */
    @Override
    protected boolean hasValidData() {
        return !latestPose.equals(new Pose2d());
    }
}
