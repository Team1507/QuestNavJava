// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.utilities.RRTGenerator;

// Robot Constants
import static frc.robot.Constants.MoveToPose.*;
import static frc.robot.Constants.Speed.*;

/**
 * Command that drives the robot along a generated trajectory
 * from its current pose to a target pose.
 * - Uses TrajectoryFactory to generate a safe path (with obstacle checks).
 * - Samples trajectory over time and applies PID tracking.
 * - Includes stall detection and tolerance checks.
 */
public class CmdMoveRRT extends Command {
  /** Creates a new CmdMoveRRT. */
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

  public CmdMoveRRT(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    addRequirements(drivetrain);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset PID controllers
    xController.reset();
    yController.reset();
    thetaController.reset();

    // Generate trajectory from current pose → target pose
    Pose2d currentPose = drivetrain.getState().Pose;
    trajectory = RRTGenerator.generateTrajectoryRRT(currentPose, targetPose);

    // Start timer for trajectory sampling
    timer.reset();
    timer.start();

    // Record initial pose for stall detection
    lastX = currentPose.getX();
    lastY = currentPose.getY();
    lastCheckTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
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
    xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), getMaxSpeed()), xSpeed);
    ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), getMaxSpeed()), ySpeed);
    thetaSpeed = Math.copySign(Math.min(Math.abs(thetaSpeed), getMaxAngularSpeed()), thetaSpeed);

    // Deadband near goal
    if (Math.abs(desiredPose.getX() - currentPose.getX()) < DEADBAND_ERROR) xSpeed = 0.0;
    if (Math.abs(desiredPose.getY() - currentPose.getY()) < DEADBAND_ERROR) ySpeed = 0.0;

    // Convert to robot-relative speeds
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, thetaSpeed, currentPose.getRotation()
    );

    drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(robotRelativeSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot when command finishes or is interrupted
    drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
        .withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)));
  }

  // Returns true when the command should end.
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
