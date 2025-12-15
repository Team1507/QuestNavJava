package frc.robot.subsystems;

import java.util.OptionalDouble;
import java.util.OptionalInt;

// WPI libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

// Vision base class
import frc.robot.subsystems.vision.VisionSystem;

// Robot Utilities
import frc.robot.utilities.Telemetry;
import frc.robot.subsystems.quest.PoseFrame;
import frc.robot.subsystems.quest.QuestNav;

// Constants
import static frc.robot.Constants.Quest.*;

/**
 * VisionSystem implementation for QuestNav.
 *
 * Responsible for:
 *  - Reading QuestNav pose frames
 *  - Transforming them into robot space
 *  - Updating latestPose for the VisionSystem base class
 *  - Providing QuestNav-specific utilities (battery %, timestamps, etc.)
 */
public class QuestNavSubsystem extends VisionSystem {

    private final QuestNav questNav = new QuestNav();

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain, Telemetry logger) {
        super(drivetrain, logger);
    }

    /**
     * Updates the latest pose from QuestNav.
     * Called automatically by VisionSystem.periodic().
     */
    @Override
    protected void update() {
        // If QuestNav is not tracking, clear pose and exit
        if (!questNav.isTracking()) {
            latestPose = new Pose2d();
            return;
        }

        // Get all unread frames
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
        if (questFrames.length == 0) return;

        // Use the most recent frame
        PoseFrame questFrame = questFrames[questFrames.length - 1];
        Pose3d questPose = questFrame.questPose3d();

        // Transform QuestNav pose into robot coordinate space
        Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
        latestPose = robotPose.toPose2d();

        // Fuse QuestNav into drivetrain estimator:
        // drivetrain.addVisionMeasurement(latestPose, questFrame.dataTimestamp(), QUESTNAV_STD_DEVS);
    }

    /**
     * Returns whether QuestNav currently has valid tracking data.
     */
    @Override
    protected boolean hasValidData() {
        return questNav.isTracking();
    }

    // -------------------------
    // QuestNav-specific helpers
    // -------------------------

    public boolean isConnected() {
        return questNav.isConnected();
    }

    public OptionalInt getBatteryPercent() {
        return questNav.getBatteryPercent();
    }

    public OptionalDouble getAppTimestamp() {
        return questNav.getAppTimestamp();
    }

    /**
     * Sets the internal QuestNav pose (useful when integrating PhotonVision).
     */
    public void setQuestNavPose(Pose3d pose) {
        questNav.setPose(pose);
    }
}
