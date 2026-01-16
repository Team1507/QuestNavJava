package frc.robot;

// CTRE libraries
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;

// WPI libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Robot Commands
import frc.robot.commands.CmdMoveRRT;
import frc.robot.commands.CmdShooterPIDTuner;

// Robot Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// Robot Constants
import static frc.robot.Constants.IO.*;
import static frc.robot.Constants.Speed.*;
import static frc.robot.Constants.Shooter.*;

// Shooter model imports
import frc.robot.shooter.data.PoseSupplier;
import frc.robot.shooter.data.ShotTrainer;
import frc.robot.shooter.model.ModelLoader;
import frc.robot.shooter.model.ShooterModel;

// Nodes
import frc.robot.navigation.Nodes.Hub;
import frc.robot.navigation.Nodes.AllianceZoneBlue;
import frc.robot.navigation.Nodes.AllianceZoneRed;

// Autos
import frc.robot.auto.routines.OnePieceAuto;

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
                Hub.CENTER // default target
            );

        public final ShotTrainer shotTrainer =
            new ShotTrainer(
                shooterSubsystem.getShooterMotor(),
                poseSupplier,
                Hub.CENTER.getTranslation()
            );    

    // -----------------------------
    // Autos
    // -----------------------------
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureShooterDefault();
        configureAutoChooser();
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
        joystick.a().onTrue(new CmdMoveRRT(drivetrain, Hub.APPROACH_FRONT));

        joystick.b().onTrue(drivetrain.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));

        // Shooter Target Update
        // Right bumper → shoot at HUB_POSE
        joystick.rightBumper().onTrue(
            Commands.runOnce(() -> shooterSubsystem.setTargetPose(Hub.APPROACH_FRONT))
        );

        // Right trigger → shoot at SHOT_A_POSE
        joystick.rightTrigger().onTrue(
            Commands.runOnce(() -> shooterSubsystem.setTargetPose(AllianceZoneBlue.LEFT))
        );

        // Left trigger → shoot at SHOT_B_POSE
        joystick.leftTrigger().onTrue(
            Commands.runOnce(() -> shooterSubsystem.setTargetPose(AllianceZoneBlue.RIGHT))
        );

        // Shooter sim visualization
        joystick.rightBumper().onTrue(
            new FuelSimulator(shooterSubsystem, poseSupplier, shotTrainer, Hub.APPROACH_FRONT.getTranslation())
        );
        joystick.rightTrigger().onTrue(
            new FuelSimulator(shooterSubsystem, poseSupplier, shotTrainer, AllianceZoneBlue.LEFT.getTranslation())
        );
        joystick.leftTrigger().onTrue(
            new FuelSimulator(shooterSubsystem, poseSupplier, shotTrainer, AllianceZoneBlue.RIGHT.getTranslation())
        );

        // Vision pose reset
        joystick.y().onTrue(
            Commands.runOnce(photonVisionSubsystem::resetDrivetrainToVisionPose)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        // PID Tuner
        SmartDashboard.putData( 
            "Run Shooter PID Tuner",
            new CmdShooterPIDTuner(shooterSubsystem, MAX_RPM) // max RPM here
        );
    }

    private void configureAutoChooser() {

        // Default auto
        autoChooser.setDefaultOption(
            "Auto Do Nothing",
            Commands.print("Doing nothing")
        );
    
        // Example autos using your new builder
        autoChooser.addOption(
            "One Piece Auto",
            OnePieceAuto.build(drivetrain)
        );
    
        // Publish to dashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
    }    

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }    

    private double applyDeadband(double value, double deadband) {
        return (Math.abs(value) < deadband) ? 0.0 : value;
    }
}
