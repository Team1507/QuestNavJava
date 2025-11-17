package frc.robot.subsystems.quest;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Subsystem wrapper for QuestNav integration.
 * Responsible for reading QuestNav pose frames, transforming them into robot space,
 * and feeding them into the drivetrain's pose estimator.
 */

public class QuestNavSubsystem extends SubsystemBase {

    private final QuestNav questNav = new QuestNav();
    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d latestPose = new Pose2d();

    // Mount transform: robot origin -> QuestNav sensor
    private static final Transform3d ROBOT_TO_QUEST =
        new Transform3d(0.381, 0.0, 0.3048, new Rotation3d(0.0, 0.0, 0.0));

    // Standard deviations for measurement trust
    private static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
        0.02,  // 2 cm X
        0.02,  // 2 cm Y
        0.035  // ~2 degrees
    );

    // Publisher for debugging
    private final StructPublisher<Pose2d> questPosePublisher =
        NetworkTableInstance.getDefault().getStructTopic("QuestPose", Pose2d.struct).publish();

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        update();
    }

    private void update() {
        if (!questNav.isTracking()) return;
    
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
        for (PoseFrame questFrame : questFrames) {
            Pose3d questPose = questFrame.questPose3d();
            double timestamp = questFrame.dataTimestamp();
    
            Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
            latestPose = robotPose.toPose2d(); // <-- update latestPose
    
            drivetrain.addVisionMeasurement(latestPose, timestamp, QUESTNAV_STD_DEVS);
            questPosePublisher.set(latestPose);
        }
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
}
