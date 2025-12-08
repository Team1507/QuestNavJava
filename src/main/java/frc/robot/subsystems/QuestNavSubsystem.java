package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

// WPI libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Robot Utilities
import frc.robot.utilities.Telemetry;
import frc.robot.subsystems.quest.PoseFrame;
import frc.robot.subsystems.quest.QuestNav;

// Constants
import static frc.robot.Constants.Quest.*;

/**
 * Subsystem wrapper for QuestNav integration.
 * Responsible for reading QuestNav pose frames, transforming them into robot space,
 * and feeding them into the drivetrain's pose estimator.
 */
public class QuestNavSubsystem extends SubsystemBase {

    private final QuestNav questNav = new QuestNav();
    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d latestPose = new Pose2d();

    // Publisher for debugging
    private final Telemetry logger;

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain, Telemetry logger) {
        this.drivetrain = drivetrain;
        this.logger = logger;
    }

    @Override
    public void periodic() {
        update();
    }

    private void update() {
        if (!questNav.isTracking()) {
            latestPose = new Pose2d(); // clear when not tracking
            return;
        }
    
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
        if (questFrames.length == 0) return;
    
        // Use the most recent frame
        PoseFrame questFrame = questFrames[questFrames.length - 1];
        Pose3d questPose = questFrame.questPose3d();
        double timestamp = questFrame.dataTimestamp();
    
        Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
        latestPose = robotPose.toPose2d();
    
        //drivetrain.addVisionMeasurement(latestPose, timestamp, QUESTNAV_STD_DEVS);

        // Publish to Telemetry logger as well
        logger.publishQuestPose(latestPose);
    }
    
    /** Returns the most recent robot pose from QuestNav */
    public Optional<Pose2d> getLatestPose() {
        return questNav.isTracking() ? Optional.of(latestPose) : Optional.empty();
    }

    /** Returns whether QuestNav is actively tracking */
    public boolean isTracking() {
        return questNav.isTracking();
    }

    public boolean isConnected() {
        return questNav.isConnected();
    }

    public OptionalInt getBatteryPercent() {
        return questNav.getBatteryPercent();
    }

    public OptionalDouble getAppTimestamp() {
        return questNav.getAppTimestamp();
    }

    public void setDrivetrainPose(){
        /*
         * This function will reset the current pose of the
         * drivetrain to the latest pose acquired from the
         * QuestNav system.
         */
        drivetrain.resetPose(latestPose);
    }

    public void setQuestNavPose(Pose3d pose){
        /*
         * The function will take in a 3D pose and set the
         * QuestNav pose to that pose. Useful if integrating
         * PhotonVision or another outside source.
         */
        questNav.setPose(pose);
    }
}
