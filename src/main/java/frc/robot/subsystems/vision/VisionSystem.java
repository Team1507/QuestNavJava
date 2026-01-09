// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

// WPI libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Robot Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Robot Utilities
import frc.robot.utilities.Telemetry;

/**
 * Base class for all robot vision systems (QuestNav, PhotonVision, etc.).
 *
 * Responsibilities:
 *  - Maintain the most recent valid vision pose
 *  - Track pose validity and freshness
 *  - Provide a safe API for consumers (drivetrain, commands)
 *  - Publish telemetry consistently
 *
 * Subclasses should:
 *  - Call acceptPose(...) when a new valid pose is produced
 *  - Call invalidatePose() when tracking is lost
 */
public abstract class VisionSystem extends SubsystemBase {

    /** Most recent accepted vision pose. */
    protected Pose2d latestPose = new Pose2d();

    /** Whether the current pose is valid. */
    private boolean poseValid = false;

    /** FPGA timestamp (seconds) when the pose was last updated. */
    private double lastPoseTimestamp = 0.0;

    /** Maximum age (seconds) before a pose is considered stale. */
    private static final double MAX_POSE_AGE_SEC = 0.25;

    /** Reference to the drivetrain for odometry resets. */
    protected final CommandSwerveDrivetrain drivetrain;

    /** Telemetry logger for AdvantageScope / dashboards. */
    protected final Telemetry logger;

    protected VisionSystem(CommandSwerveDrivetrain drivetrain, Telemetry logger) {
        this.drivetrain = drivetrain;
        this.logger = logger;
    }

    /**
     * Called once per robot loop.
     * Subclasses update pose state; base class handles telemetry.
     */
    @Override
    public final void periodic() {
        update();
        addVisionMeasurementToDrivetrain();
        publishTelemetry();
    }

    /**
     * Subclasses implement their vision update logic here.
     * They should call acceptPose(...) or invalidatePose().
     */
    protected abstract void update();
    protected abstract void addVisionMeasurementToDrivetrain();

    // ---------------------------------------------------------------------
    // Pose lifecycle helpers (used by subclasses)
    // ---------------------------------------------------------------------

    /**
     * Accepts a newly computed vision pose.
     *
     * @param pose The estimated robot pose.
     * @param timestampSeconds Timestamp associated with the measurement.
     */
    protected void acceptPose(Pose2d pose, double timestampSeconds) {
        latestPose = pose;
        lastPoseTimestamp = timestampSeconds;
        poseValid = true;
    }

    /**
     * Marks the current pose as invalid (e.g., no targets visible).
     */
    protected void invalidatePose() {
        poseValid = false;
    }

    // ---------------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------------

    /**
     * Returns the most recent valid and fresh vision pose.
     */
    public Optional<Pose2d> getLatestPose() {
        if (!poseValid) {
            return Optional.empty();
        }

        double age = Timer.getFPGATimestamp() - lastPoseTimestamp;
        if (age > MAX_POSE_AGE_SEC) {
            return Optional.empty();
        }

        return Optional.of(latestPose);
    }

    /**
     * Returns whether this vision system currently has usable tracking data.
     */
    public boolean isTracking() {
        return getLatestPose().isPresent();
    }

    /**
     * Resets drivetrain odometry to the latest valid vision pose.
     * Safe to call from a button binding.
     */
    public void resetDrivetrainToVisionPose() {
        getLastKnownVisionPose().ifPresent(pose -> {
            drivetrain.resetPose(pose);
            System.out.println(
                "[VisionSystem] HARD reset drivetrain pose from " +
                getClass().getSimpleName() + ": " + pose
            );
        });
    }    

    public double getLastPoseTimestamp() {
        return lastPoseTimestamp;
    }
    
    public Optional<Pose2d> getLastAcceptedPose() {
        return poseValid ? Optional.of(latestPose) : Optional.empty();
    }

    public Optional<Pose2d> getLastKnownVisionPose() {
        return lastPoseTimestamp > 0 ? Optional.of(latestPose) : Optional.empty();
    }    
    
    /**
     * Publishes the latest pose to Telemetry under a subsystem-specific topic.
     * The topic name is derived from the subclass's class name.
     */
    protected void publishTelemetry() {
        if (logger != null) {
            logger.publishVisionPose(getClass().getSimpleName(), latestPose);
        }
    }
}