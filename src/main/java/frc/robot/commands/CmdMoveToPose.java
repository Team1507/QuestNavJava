package frc.robot.commands;

// WPI libraries
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
// Robot Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
// Robot Constants
import static frc.robot.Constants.MoveToPose.*;

/**
 * Command that drives the robot to a target Pose2d (x, y, heading).
 * - Uses PID controllers for X, Y, and rotation.
 * - Sends velocity requests to the CTRE swerve drivetrain via ApplyRobotSpeeds.
 * - Includes stall detection logic to exit if the robot gets stuck.
 */
public class CmdMoveToPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Pose2d targetPose;

  // Variables used for stall detection
  private double lastX, lastY, lastCheckTime;

  // PID controllers for each axis of motion
  // Gains and tolerances are defined in Constants.MoveToPose
  private final PIDController xController = new PIDController(X_KP, X_KI, X_KD);
  private final PIDController yController = new PIDController(Y_KP, Y_KI, Y_KD);
  private final PIDController thetaController = new PIDController(THETA_KP, THETA_KI, THETA_KD);

  public CmdMoveToPose(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    addRequirements(drivetrain); // ensures no other command uses drivetrain at same time

    // Allow rotation PID to wrap around [-pi, pi] so it chooses shortest turn
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    // Record the robot's starting position for stall detection
    lastX = drivetrain.getState().Pose.getX();
    lastY = drivetrain.getState().Pose.getY();
    lastCheckTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    // Reset PID controllers at start of command
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    // Get current robot pose from drivetrain's state estimator
    Pose2d currentPose = drivetrain.getState().Pose;

    // Calculate velocity commands using PID controllers
    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double thetaSpeed = thetaController.calculate(
        currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians()
    );

    // Cap speeds
    xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), MAX_LINEAR_SPEED), xSpeed);
    ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), MAX_LINEAR_SPEED), ySpeed);
    thetaSpeed = Math.copySign(Math.min(Math.abs(thetaSpeed), MAX_ANGULAR_SPEED), thetaSpeed);

    // Deadband small dithers near target
    if (Math.abs(targetPose.getX() - currentPose.getX()) < DEADBAND_ERROR) xSpeed = 0.0;
    if (Math.abs(targetPose.getY() - currentPose.getY()) < DEADBAND_ERROR) ySpeed = 0.0;

    // Convert field-relative X/Y to robot-relative using current heading
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed,
        ySpeed,
        thetaSpeed,
        currentPose.getRotation()
    );

    // Send robot-relative speeds to CTRE
    SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds()
        .withSpeeds(robotRelativeSpeeds);

    drivetrain.setControl(request);
  }


  @Override
  public void end(boolean interrupted) {
    // Stop the robot when command finishes or is interrupted
    drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
        .withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)));
  }

  @Override
  public boolean isFinished() {
    // Get current pose and current time
    Pose2d currentPose = drivetrain.getState().Pose;
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    // --- Stall detection logic ---
    // Check how far the robot has moved since last check
    double dx = Math.abs(currentPose.getX() - lastX);
    double dy = Math.abs(currentPose.getY() - lastY);

    // If movement is below threshold for longer than STALL_TIMEOUT, exit
    if (dx < STALL_THRESHOLD && dy < STALL_THRESHOLD && (now - lastCheckTime) > STALL_TIMEOUT) {
        return true; // robot is stuck
    }

    // If robot has moved enough, update last check values
    if (dx > STALL_THRESHOLD || dy > STALL_THRESHOLD) {
        lastX = currentPose.getX();
        lastY = currentPose.getY();
        lastCheckTime = now;
    }

    // --- Normal finish condition ---
    // End when robot is within position and angle tolerances of target
    boolean atPosition = Math.abs(currentPose.getX() - targetPose.getX()) < POSITION_TOLERANCE_METERS &&
                         Math.abs(currentPose.getY() - targetPose.getY()) < POSITION_TOLERANCE_METERS;
    boolean atAngle = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians())
                      < ANGLE_TOLERANCE_RADIANS;

    return atPosition && atAngle;
  }
}
