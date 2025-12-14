// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

// WPI libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Robot Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Robot Utilities
import frc.robot.utilities.Telemetry;

/**
 * Base class for all robot vision systems (QuestNav, PhotonVision, etc.).
 *
 * This abstraction provides:
 *  - A shared pose cache (latestPose)
 *  - A unified periodic update loop
 *  - A consistent API for checking tracking status
 *  - A standard method for resetting drivetrain pose
 *  - Automatic telemetry publishing through Telemetry.java
 *
 * Subclasses only need to implement:
 *  - update(): how the system produces a new pose
 *  - hasValidData(): whether the pose is trustworthy
 *
 * This keeps all vision subsystems consistent, predictable, and easy to extend.
 */
public abstract class VisionSystem extends SubsystemBase {

    /** The most recent valid robot pose produced by this vision system. */
    protected Pose2d latestPose = new Pose2d();

    /** Reference to the drivetrain so vision systems can reset odometry. */
    protected final CommandSwerveDrivetrain drivetrain;

    /** Telemetry logger for publishing poses to AdvantageScope. */
    protected final Telemetry logger;

    /**
     * Construct a new VisionSystem.
     *
     * @param drivetrain The drivetrain whose pose may be reset by this system.
     * @param logger     Telemetry logger for publishing vision poses.
     */
    public VisionSystem(CommandSwerveDrivetrain drivetrain, Telemetry logger) {
        this.drivetrain = drivetrain;
        this.logger = logger;
    }

    /**
     * Called once per robot loop.
     * Subclasses update their pose, then the base class publishes telemetry.
     */
    @Override
    public void periodic() {
        update();
        publishTelemetry();
    }

    /**
     * Subclasses implement how they update latestPose.
     * This may involve reading sensors, processing frames, or filtering data.
     */
    protected abstract void update();

    /**
     * Subclasses implement whether the current pose is valid and trustworthy.
     * Used by getLatestPose() and isTracking().
     */
    protected abstract boolean hasValidData();

    /**
     * Returns the most recent valid pose from this vision system.
     *
     * @return Optional containing the pose if tracking, otherwise empty.
     */
    public Optional<Pose2d> getLatestPose() {
        return hasValidData() ? Optional.of(latestPose) : Optional.empty();
    }

    /**
     * Returns whether this vision system currently has valid tracking data.
     */
    public boolean isTracking() {
        return hasValidData();
    }

    /**
     * Resets the drivetrain's odometry to the latest vision pose.
     * Only applies if the pose is valid.
     */
    public void setDrivetrainPose() {
        getLatestPose().ifPresent(drivetrain::resetPose);
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