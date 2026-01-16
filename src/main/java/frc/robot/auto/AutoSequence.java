package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;

// Robot Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Robot Commands
import frc.robot.commands.CmdMoveRRT;

/**
 * AutoSequence
 *
 * Mentor-provided base class for building autonomous routines using a fluent API.
 * Students will extend this by adding new high-level actions (score, intake, etc.)
 * that internally add Commands to the 'steps' list.
 */
public class AutoSequence {

    // Internal list of steps that will be executed in order
    private final List<Command> steps = new ArrayList<>();

    // Required subsystem reference for movement commands
    private final CommandSwerveDrivetrain drivetrain;

    public AutoSequence(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /**
     * Move the robot to a target Pose2d using RRT path planning.
     * Students do NOT modify this method.
     */
    public AutoSequence moveTo(Pose2d target) {
        steps.add(new CmdMoveRRT(drivetrain, target));
        return this;
    }

    /**
     * Build the final autonomous command sequence.
     * Students do NOT modify this method.
     */
    public Command build() {
        return Commands.sequence(steps.toArray(Command[]::new));
    }

    /**
     * TODO: Add a scoring action.
     * This should add a Command that performs the robot's scoring routine.
     * Example:
     * steps.add(new ScoreCommand(shooterSubsystem));
     */
    public AutoSequence score() {
        // TODO: Implement scoring command
        return this;
    }

    /**
     * TODO: Add an intake action.
     * This should add a Command that runs the intake to collect a game piece.
     * Example:
     * steps.add(new IntakeCommand(intakeSubsystem));
     */
    public AutoSequence intake() {
        // TODO: Implement intake command
        return this;
    }

    /**
     * Wait for a number of seconds before continuing.
     * This is already implemented for students.
     */
    public AutoSequence waitSeconds(double seconds) {
        steps.add(Commands.waitSeconds(seconds));
        return this;
    }

    // may add more actions here:
    // - shoot()
    // - moveThrough(Pose2d...)
    // - parallel(Command...)
    // - race(Command...)
    // - outtake()
    // - align()
    // etc.
}
