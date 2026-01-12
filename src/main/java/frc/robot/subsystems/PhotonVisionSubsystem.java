package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

// WPI libraries
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
 * Responsibilities:
 *  - Read AprilTag detections from PhotonVision
 *  - Run PhotonPoseEstimator
 *  - Accept or invalidate poses via VisionSystem helpers
 *  - Publish diagnostics safely
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
    public PhotonVisionSubsystem(
        CommandSwerveDrivetrain drivetrain,
        Telemetry logger,
        String cameraName
    ) {
        super(drivetrain, logger);

        this.camera = new PhotonCamera(cameraName);

        this.poseEstimator = new PhotonPoseEstimator(
            APRILTAG_LAYOUT,
            PoseStrategy.LOWEST_AMBIGUITY,
            CAMERA_TO_ROBOT
        );
    }

    /**
     * Updates the vision pose.
     * Called automatically by VisionSystem.periodic().
     */
    @Override
    protected void update() {

        PhotonPipelineResult result = camera.getLatestResult();

        // ------------------------------------------------------------
        // Basic diagnostics
        // ------------------------------------------------------------
        SmartDashboard.putBoolean("PhotonVision/HasTarget", result.hasTargets());
        SmartDashboard.putNumber("PhotonVision/TagCount", result.getTargets().size());

        // ------------------------------------------------------------
        // Per‑target debug info (safe even if empty)
        // ------------------------------------------------------------
        for (var t : result.getTargets()) {
            SmartDashboard.putNumber("Tag " + t.getFiducialId() + "/Yaw", t.getYaw());
            SmartDashboard.putNumber("Tag " + t.getFiducialId() + "/Pitch", t.getPitch());
            SmartDashboard.putNumber("Tag " + t.getFiducialId() + "/Area", t.getArea());
        }

        // ------------------------------------------------------------
        // Ambiguity (guarded)
        // ------------------------------------------------------------
        if (result.hasTargets() && result.getBestTarget() != null) {
            SmartDashboard.putNumber(
                "PhotonVision/BestAmbiguity",
                result.getBestTarget().getPoseAmbiguity()
            );
        } else {
            SmartDashboard.putNumber("PhotonVision/BestAmbiguity", -1);
        }

        // ------------------------------------------------------------
        // Pose estimation
        // ------------------------------------------------------------
        var estimated = poseEstimator.update(result);
        SmartDashboard.putBoolean("PhotonVision/PoseEstimated", estimated.isPresent());

        if (estimated.isEmpty()) {
            invalidatePose();
            return;
        }

        EstimatedRobotPose pose = estimated.get();

        // ------------------------------------------------------------
        // Accept pose
        // ------------------------------------------------------------
        acceptPose(
            pose.estimatedPose.toPose2d(),
            pose.timestampSeconds
        );
    }

    public void addVisionMeasurementToDrivetrain() {
        getLatestPose().ifPresent(pose -> {
            drivetrain.addVisionMeasurement(
                pose,
                getLastPoseTimestamp(),
                PHOTONVISION_STD_DEVS
            );
        });
    }
}
