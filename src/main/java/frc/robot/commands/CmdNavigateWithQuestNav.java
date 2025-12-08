package frc.robot.commands;

// WPI libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
// Robot Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;
// Robot Constants
// Robot Utilities
import frc.robot.utilities.TrajectoryFactory;

public class CmdNavigateWithQuestNav extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final QuestNavSubsystem questNav;
    private final Pose2d targetPose;

    // Trajectory to follow (generated at runtime based on current pose + field map)
    private Trajectory trajectory;

    // Controller to follow trajectory
    private final HolonomicDriveController controller =
        new HolonomicDriveController(
            new PIDController(1.0, 0.0, 0.0), // X
            new PIDController(1.0, 0.0, 0.0), // Y
            new ProfiledPIDController(2.0, 0.0, 0.0, 
                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                    Math.PI, Math.PI)) // Rotation
        );

    public CmdNavigateWithQuestNav(CommandSwerveDrivetrain drivetrain,
                                   QuestNavSubsystem questNav,
                                   Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.questNav = questNav;
        this.targetPose = targetPose;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // 1. Get current pose from QuestNav (absolute field position)
        Pose2d currentPose = questNav.getLatestPose().orElse(drivetrain.getState().Pose);

        // 2. Generate trajectory from currentPose → targetPose
        //    (In practice, you’d use WPILib TrajectoryGenerator or PathPlannerLib,
        //     possibly with waypoints to avoid obstacles.)
        trajectory = TrajectoryFactory.generateTrajectory(currentPose, targetPose);
    }

    @Override
    public void execute() {
        // 3. Sample trajectory at current time
        double elapsed = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        var desiredState = trajectory.sample(elapsed);

        // 4. Get current pose from QuestNav
        Pose2d currentPose = questNav.getLatestPose().orElse(drivetrain.getState().Pose);

        // 5. Calculate chassis speeds to follow trajectory
        var speeds = controller.calculate(currentPose, desiredState, targetPose.getRotation());

        // 6. Send speeds to drivetrain (via CTRE ApplyRobotSpeeds)
        drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds()
            .withSpeeds(speeds));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop robot
        drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds()
            .withSpeeds(new edu.wpi.first.math.kinematics.ChassisSpeeds()));
    }

    @Override
    public boolean isFinished() {
        // End when trajectory is complete
        return trajectory != null &&
               edu.wpi.first.wpilibj.Timer.getFPGATimestamp() > trajectory.getTotalTimeSeconds();
    }
}
