// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// PhotonVision Libraries
import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

// WPI libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

// Robot Constants (your own constants class defines APRILTAG_LAYOUT and CAMERA_TO_ROBOT)
import static frc.robot.Constants.IO.*;
import static frc.robot.Constants.FieldElements.*;

public class PhotonVisionSubsystem extends SubsystemBase {
    // Camera object that talks to PhotonVision running on the coprocessor (OrangePi)
    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;

    // Pose estimator that uses AprilTag detections to compute robot field position
    private final PhotonPoseEstimator poseEstimator;

    /**
     * Constructor for the PhotonVisionSubsystem.
     * @param cameraName The name of the camera as configured in the PhotonVision UI.
     */
    public PhotonVisionSubsystem(CommandSwerveDrivetrain drivetrain, String cameraName) {
        // Create a PhotonCamera instance bound to the given camera name
        this.drivetrain = drivetrain;
        this.camera = new PhotonCamera(cameraName);

        // Create the pose estimator using the official AprilTag field layout,
        // a chosen strategy, and the known transform from robot center to camera.
        // MULTI_TAG_PNP_ON_COPROCESSOR = use all visible tags to compute pose on the coprocessor.
        // LOWEST_AMBIGUITY = pick the mathematically least ambiguous solution.
        this.poseEstimator = new PhotonPoseEstimator(
            APRILTAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            CAMERA_TO_ROBOT
        );
    }

    /**
     * Query PhotonVision for the robot's estimated field pose.
     * @return An Optional containing the estimated Pose2d if tags are visible,
     *         or Optional.empty() if no valid estimate is available.
     */
    public Optional<Pose2d> getEstimatedPose() {
        // Ask the camera for its latest processed frame results
        PhotonPipelineResult result = camera.getLatestResult();
    
        // Feed those results into the pose estimator to compute a robot pose
        Optional<EstimatedRobotPose> estimated = poseEstimator.update(result);
    
        // If PhotonVision produced a valid estimate, convert it to a WPILib Pose2d
        if (estimated.isPresent()) {
            return Optional.of(estimated.get().estimatedPose.toPose2d());
        }

        // Otherwise, return empty to signal "no tags visible / no estimate available"
        return Optional.empty();
    }

    /**
     * Reset the drivetrain pose to the latest PhotonVision estimate,
     * but only if multiple AprilTags are visible (for reliability).
     */
    public void setDrivetrainPose() {
        // Get the latest pipeline result from the camera
        PhotonPipelineResult result = camera.getLatestResult();

        // Only proceed if PhotonVision sees at least 2 tags
        if (result.hasTargets() && result.getTargets().size() >= 2) {
            Optional<EstimatedRobotPose> estimated = poseEstimator.update(result);

            if (estimated.isPresent()) {
                // Reset drivetrain odometry to PhotonVision's absolute field pose
                drivetrain.resetPose(estimated.get().estimatedPose.toPose2d());
            }
        }
        // If fewer than 2 tags are visible, do nothing (avoid unreliable reset)
    }
}