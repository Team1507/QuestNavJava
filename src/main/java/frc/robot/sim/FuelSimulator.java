package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.shooter.data.PoseSupplier;
import frc.robot.shooter.data.ShotTrainer;

import static frc.robot.Constants.FieldElements.*;

public class FuelSimulator extends Command {

    private final ShooterSubsystem shooter;
    private final PoseSupplier poseSupplier;
    private final ShotTrainer shotTrainer;

    private final Timer timer = new Timer();

    private final NetworkTableEntry fuelPublisher =
        NetworkTableInstance.getDefault()
            .getTable("Sim")
            .getEntry("FuelPose");

    public FuelSimulator(
        ShooterSubsystem shooter,
        PoseSupplier poseSupplier,
        ShotTrainer shotTrainer
    ) {
        this.shooter = shooter;
        this.poseSupplier = poseSupplier;
        this.shotTrainer = shotTrainer;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        // Notify ShotTrainer that a shot occurred
        shotTrainer.notifyShotFired(Timer.getFPGATimestamp());

        Pose2d robotPose = poseSupplier.getPose();

        // Compute direction toward hub
        Translation2d toHub = HUB_POSE.minus(robotPose.getTranslation());
        double norm = toHub.getNorm();
        Translation2d direction = (norm > 1e-6)
            ? new Translation2d(toHub.getX() / norm, toHub.getY() / norm)
            : new Translation2d(0, 0);

        // Convert RPM → landing distance
        double rpm = shooter.getTargetRPM();
        double speed = rpm * 0.0012;     // m/s
        double flightTime = 1.0;         // tune this constant
        double distance = speed * flightTime;

        // Compute landing point
        Translation2d landing = robotPose.getTranslation().plus(direction.times(distance));

        // Publish landing pose (rotation irrelevant)
        fuelPublisher.setDoubleArray(new double[] {
            landing.getX(),
            landing.getY(),
            0.0
        });
    }

    @Override
    public void execute() {
        // No animation needed
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 0.5; // keep visible for 5 seconds
    }

    @Override
    public void end(boolean interrupted) {
        fuelPublisher.setDoubleArray(new double[] {});
    }
}