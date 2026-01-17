package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Shooter Model
import frc.robot.shooter.data.PoseSupplier;
import frc.robot.shooter.data.ShotRecord;
import frc.robot.shooter.model.ShooterModel;

public class ShooterSubsystem extends SubsystemBase {

    // -----------------------------
    // Hardware
    // -----------------------------
    private final TalonFX shooterMotor;
    private final VelocityVoltage velocityRequest = 
        new VelocityVoltage(0).withSlot(0);

    // -----------------------------
    // Model-driven shooter fields
    // -----------------------------
    private final ShooterModel model;
    private final PoseSupplier poseSupplier;
    private Pose2d targetPose;

    private double targetRPS = 0.0;

    // -----------------------------
    // Simulation fields
    // -----------------------------
    private boolean simulate = RobotBase.isSimulation();
;
    private double simulatedRPM = 0.0;

    public ShooterSubsystem(
        TalonFX shooterMotor,
        ShooterModel model,
        PoseSupplier poseSupplier,
        Pose2d targetPose
    ) {
        this.shooterMotor = shooterMotor;
        this.model = model;
        this.poseSupplier = poseSupplier;
        this.targetPose = targetPose;

        configurePID();
    }

    // -----------------------------
    // PID Configuration
    // -----------------------------
    private void configurePID() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        // Slot0 PID values
        cfg.Slot0.kP = 0.10;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        // Slot0 Feedforward values
        cfg.Slot0.kV = 0.12;
        cfg.Slot0.kS = 0.0;
        cfg.Slot0.kA = 0.0;
        shooterMotor.getConfigurator().apply(cfg);
    }

    public TalonFX getShooterMotor() {
        return shooterMotor;
    }

    // -----------------------------
    // Simulation toggle
    // -----------------------------
    public void setSimulate(boolean enable) {
        this.simulate = enable;
        SmartDashboard.putBoolean("Shooter Simulate", simulate);
    }

    public boolean isSimulating() {
        return simulate;
    }

    // -----------------------------
    // Shooter telemetry accessors
    // -----------------------------

    public double getShooterRPM() {
        return simulate
            ? simulatedRPM
            : shooterMotor.getVelocity().getValueAsDouble() * 60;
    }

    public double getShooterVoltage() {
        if (simulate) {
            // Fake voltage proportional to effort
            return 12.0 * Math.min(1.0, Math.abs(targetRPS) / 83.33);
        }
        return shooterMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getStatorCurrent() {
        if (simulate) {
            // Fake current: increases with RPM
            return 5.0 + (Math.abs(simulatedRPM) / 1000.0);
        }
        return shooterMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return simulate
            ? getStatorCurrent()
            : shooterMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getClosedLoopError() {
        return (targetRPS * 60) - getShooterRPM();
    }

    // -----------------------------
    // Build ShotRecord for model
    // -----------------------------
    private ShotRecord buildTelemetry() {
        Pose2d pose = poseSupplier.getPose();
        double distance = pose.getTranslation().getDistance(targetPose.getTranslation());

        return new ShotRecord(
            getShooterRPM(),
            getShooterVoltage(),
            getStatorCurrent(),
            getSupplyCurrent(),
            getClosedLoopError(),
            pose,
            distance
        );
    }

    // -----------------------------
    // Model-driven shooter update
    // -----------------------------
    public void updateShooterFromModel() {
        ShotRecord telemetry = buildTelemetry();
        double rpm = model.getRPM(telemetry);
        setTargetRPM(rpm);
    }

    // -----------------------------
    // Shooter control
    // -----------------------------
    public void setTargetRPM(double rpm) {
        // velocity control takes RPS not RPM, so need to convert RPM to RPS
        this.targetRPS = rpm / 60;
    }

    public double getTargetRPM() {
        return targetRPS * 60;
    }

    public void setTargetPose(Pose2d newTarget) {
        this.targetPose = newTarget;
    }

    // -----------------------------
    // Periodic
    // -----------------------------
    @Override
    public void periodic() {

        // Update simulation toggle from dashboard
        simulate = SmartDashboard.getBoolean("Shooter Simulate", simulate);

        if (simulate) {
            // Simple flywheel simulation: exponential approach
            double alpha = 0.10; // responsiveness factor
            simulatedRPM += ((targetRPS * 60) - simulatedRPM) * alpha;
        } else {
            // Real hardware control
            shooterMotor.setControl(velocityRequest.withVelocity(targetRPS));
        }
    }
}
