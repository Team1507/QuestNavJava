package frc.robot.commands;

// WPI libraries
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// Robot Subsystems & Utilities
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.TrajectoryFactory;

// Robot Constants
import static frc.robot.Constants.MoveToPose.*;

/**
 * Command that drives the robot along a generated trajectory
 * from its current pose to a target pose.
 * - Uses TrajectoryFactory to generate a safe path (with obstacle checks).
 * - Samples trajectory over time and applies PID tracking.
 * - Includes stall detection and tolerance checks.
 */
public class CmdMoveTrajectory extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Pose2d targetPose;

  // PID controllers for trajectory tracking
  private final PIDController xController = new PIDController(X_KP, X_KI, X_KD);
  private final PIDController yController = new PIDController(Y_KP, Y_KI, Y_KD);
  private final PIDController thetaController = new PIDController(THETA_KP, THETA_KI, THETA_KD);

  // Trajectory + timer
  private Trajectory trajectory;
  private final Timer timer = new Timer();

  // Stall detection
  private double lastX, lastY, lastCheckTime;

  public CmdMoveTrajectory(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    addRequirements(drivetrain);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    // Reset PID controllers
    xController.reset();
    yController.reset();
    thetaController.reset();

    // Generate trajectory from current pose → target pose
    Pose2d currentPose = drivetrain.getState().Pose;
    trajectory = TrajectoryFactory.generateTrajectory(currentPose, targetPose);

    // Start timer for trajectory sampling
    timer.reset();
    timer.start();

    // Record initial pose for stall detection
    lastX = currentPose.getX();
    lastY = currentPose.getY();
    lastCheckTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double t = timer.get();
    Trajectory.State goal = trajectory.sample(t);

    Pose2d desiredPose = goal.poseMeters;
    Pose2d currentPose = drivetrain.getState().Pose;

    // PID tracking of desired vs current pose
    double xSpeed = xController.calculate(currentPose.getX(), desiredPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), desiredPose.getY());
    double thetaSpeed = thetaController.calculate(
        currentPose.getRotation().getRadians(),
        desiredPose.getRotation().getRadians()
    );

    // Cap speeds
    xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), MAX_LINEAR_SPEED), xSpeed);
    ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), MAX_LINEAR_SPEED), ySpeed);
    thetaSpeed = Math.copySign(Math.min(Math.abs(thetaSpeed), MAX_ANGULAR_SPEED), thetaSpeed);

    // Deadband near goal
    if (Math.abs(desiredPose.getX() - currentPose.getX()) < DEADBAND_ERROR) xSpeed = 0.0;
    if (Math.abs(desiredPose.getY() - currentPose.getY()) < DEADBAND_ERROR) ySpeed = 0.0;

    // Convert to robot-relative speeds
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, thetaSpeed, currentPose.getRotation()
    );

    drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(robotRelativeSpeeds));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
        .withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)));
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = drivetrain.getState().Pose;
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    // Stall detection
    double dx = Math.abs(currentPose.getX() - lastX);
    double dy = Math.abs(currentPose.getY() - lastY);

    if (dx < STALL_THRESHOLD && dy < STALL_THRESHOLD && (now - lastCheckTime) > STALL_TIMEOUT) {
        return true;
    }
    if (dx > STALL_THRESHOLD || dy > STALL_THRESHOLD) {
        lastX = currentPose.getX();
        lastY = currentPose.getY();
        lastCheckTime = now;
    }

    // Finish when trajectory time is done AND within tolerances
    boolean timeDone = timer.get() > trajectory.getTotalTimeSeconds();
    boolean atPosition = Math.abs(currentPose.getX() - targetPose.getX()) < POSITION_TOLERANCE_METERS &&
                         Math.abs(currentPose.getY() - targetPose.getY()) < POSITION_TOLERANCE_METERS;
    boolean atAngle = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians())
                      < ANGLE_TOLERANCE_RADIANS;

    return timeDone && atPosition && atAngle;
  }
}
