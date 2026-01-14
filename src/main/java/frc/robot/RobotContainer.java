package frc.robot;

// CTRE libraries
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// Robot Commands
import frc.robot.commands.CmdMoveRRT;
// Robot Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.FieldElements.*;
// Robot Constants
import static frc.robot.Constants.IO.*;
import static frc.robot.Constants.MoveToPose.*;
import static frc.robot.Constants.Speed.*;

// Shooter model imports
import frc.robot.shooter.data.PoseSupplier;
import frc.robot.shooter.data.ShotTrainer;
import frc.robot.shooter.model.ModelLoader;
import frc.robot.shooter.model.ShooterModel;

// Robot Extra
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.Telemetry;
import frc.robot.sim.FuelSimulator;

public class RobotContainer {

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(getMaxSpeed());
    private final CommandXboxController joystick = new CommandXboxController(JOYSTICK_PORT);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final PhotonVisionSubsystem photonVisionSubsystem =
        new PhotonVisionSubsystem(drivetrain, logger, "Limelight_2");

    // -----------------------------
    // Shooter + Model
    // -----------------------------

    private final PoseSupplier poseSupplier = () -> drivetrain.getState().Pose;

    // Load model.json from deploy directory
    private final ShooterModel shooterModelConfig =
        ModelLoader.load("model.json", poseSupplier);

        public final ShooterSubsystem shooterSubsystem =
            new ShooterSubsystem(
                new TalonFX(SHOOTER_CAN_ID),
                shooterModelConfig,
                poseSupplier,
                HUB_POSE // default target
            );

        public final ShotTrainer shotTrainer =
            new ShotTrainer(
                shooterSubsystem.getShooterMotor(),
                poseSupplier,
                HUB_POSE
            );    

    // -----------------------------

    public RobotContainer() {
        configureBindings();
        configureShooterDefault();
    }

    /**
     * Shooter default behavior: use the trained model.json
     */
    private void configureShooterDefault() {

        shooterSubsystem.setDefaultCommand(
            Commands.run(
                () -> {
                    // Build telemetry → ask model → set RPM
                    shooterSubsystem.updateShooterFromModel();
                    //shooterSubsystem.setTargetRPM(3800);
                },
                shooterSubsystem
            )
        );
    }

    /**
     * Configures operator controls.
     */
    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                double xInput = applyDeadband(-joystick.getLeftY(), 0.15);
                double yInput = applyDeadband(-joystick.getLeftX(), 0.15);
                double rotInput = applyDeadband(-joystick.getRightX(), 0.15);

                return drive
                    .withDeadband(getMaxSpeed() * 0.1)
                    .withRotationalDeadband(getMaxAngularSpeed() * 0.1)
                    .withVelocityX(xInput * getTranslationScale() * getMaxSpeed())
                    .withVelocityY(yInput * getTranslationScale() * getMaxSpeed())
                    .withRotationalRate(rotInput * getRotationScale() * getMaxAngularSpeed());
            })
        );

        // SysId routines
        joystick.back().and(joystick.y())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading
        joystick.leftBumper()
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Motion
        joystick.a().onTrue(new CmdMoveRRT(drivetrain, POSE_A));

        joystick.b().onTrue(drivetrain.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));

        // Shooter Target Update
        // Right bumper → shoot at HUB_POSE
        joystick.rightBumper().onTrue(
            Commands.runOnce(() -> shooterSubsystem.setTargetPose(HUB_POSE))
        );

        // Right trigger → shoot at SHOT_A_POSE
        joystick.rightTrigger().onTrue(
            Commands.runOnce(() -> shooterSubsystem.setTargetPose(SHOT_A_POSE))
        );

        // Left trigger → shoot at SHOT_B_POSE
        joystick.leftTrigger().onTrue(
            Commands.runOnce(() -> shooterSubsystem.setTargetPose(SHOT_B_POSE))
        );

        // Shooter sim visualization
        joystick.rightBumper().onTrue(
            new FuelSimulator(shooterSubsystem, poseSupplier, shotTrainer, HUB_POSE)
        );
        joystick.rightTrigger().onTrue(
            new FuelSimulator(shooterSubsystem, poseSupplier, shotTrainer, SHOT_A_POSE)
        );
        joystick.leftTrigger().onTrue(
            new FuelSimulator(shooterSubsystem, poseSupplier, shotTrainer, SHOT_B_POSE)
        );

        // Vision pose reset
        joystick.y().onTrue(
            Commands.runOnce(photonVisionSubsystem::resetDrivetrainToVisionPose)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private double applyDeadband(double value, double deadband) {
        return (Math.abs(value) < deadband) ? 0.0 : value;
    }
}
