package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Shooter Model
import frc.robot.shooter.data.PoseSupplier;
import frc.robot.shooter.data.ShotRecord;
import frc.robot.shooter.model.ShooterModel;

// Constants
import static frc.robot.Constants.FieldElements.*;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterMotor;
    private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

    private final ShooterModel model;
    private final PoseSupplier poseSupplier;

    private double targetRPM = 0.0;

    public ShooterSubsystem(TalonFX shooterMotor, ShooterModel model, PoseSupplier poseSupplier) {
        this.shooterMotor = shooterMotor;
        this.model = model;
        this.poseSupplier = poseSupplier;
    }

    public TalonFX getShooterMotor() {
        return shooterMotor;
    }

    // -----------------------------
    // Shooter telemetry accessors
    // -----------------------------

    public double getShooterRPM() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    public double getShooterVoltage() {
        return shooterMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return shooterMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return shooterMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getClosedLoopError() {
        return shooterMotor.getClosedLoopError().getValueAsDouble();
    }

    // -----------------------------
    // Build ShotRecord for model
    // -----------------------------

    private ShotRecord buildTelemetry() {
        return new ShotRecord(
            getShooterRPM(),
            getShooterVoltage(),
            getStatorCurrent(),
            getSupplyCurrent(),
            getClosedLoopError(),
            poseSupplier.getPose(),
            poseSupplier.getPose().getTranslation().getDistance(HUB_POSE)
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
        this.targetRPM = rpm;
        shooterMotor.setControl(velocityRequest.withVelocity(rpm));
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    // -----------------------------
    // Periodic
    // -----------------------------

    @Override
    public void periodic() {
        shooterMotor.setControl(velocityRequest.withVelocity(targetRPM));
    }
}
