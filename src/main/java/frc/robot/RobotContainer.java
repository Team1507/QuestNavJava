// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// CTRE libraries
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// Robot Commands
import frc.robot.commands.CmdMoveToPose;
import frc.robot.commands.CmdMoveTrajectory;
import frc.robot.commands.CmdMoveRRT;
// Robot Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
// Robot Constants
import static frc.robot.Constants.IO.*;
import static frc.robot.Constants.MoveToPose.*;
import static frc.robot.Constants.Speed.*;
// Robot Extra
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.Telemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(getMaxSpeed());

    private final CommandXboxController joystick = new CommandXboxController(JOYSTICK_PORT);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem(drivetrain, logger);

    public final PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem(drivetrain, logger, null);

    
    public RobotContainer() {
        configureBindings();
    }

    /**
     * Configures operator controls.
     */
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> 
                drive
                    .withDeadband(getMaxSpeed() * 0.1)
                    .withRotationalDeadband(getMaxAngularSpeed() * 0.1)
                    .withVelocityX(-joystick.getLeftY() * getTranslationScale() * getMaxSpeed())
                    .withVelocityY(-joystick.getLeftX() * getTranslationScale() * getMaxSpeed())
                    .withRotationalRate(-joystick.getRightX() * getRotationScale() * getMaxAngularSpeed())
            )
        );


        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Start moving to POSE_A when X is pressed 
        // checking for collisions and 2 waypoints to avoid obstacles
        joystick.x().onTrue(new CmdMoveTrajectory(drivetrain, POSE_A));

        // Start moving to POSE_A when A is pressed 
        // using RRT & RRT* to plan an optimized path around obstacles
        joystick.a().onTrue(new CmdMoveRRT(drivetrain, POSE_A));

        // Start moving to POSE_A when Y is pressed 
        // using point to point motion in a stright line
        joystick.y().onTrue(new CmdMoveToPose(drivetrain, POSE_A));
        //joystick.x().onTrue(new CmdMoveToPose(drivetrain, POSE_B));


        // Cancel the move when B is pressed
        joystick.b().onTrue(drivetrain.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));

        // Set the robot pose to what PhotonVision reads
        // joystick.y().onTrue(Commands.runOnce(photonVisionSubsystem::setDrivetrainPose));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
