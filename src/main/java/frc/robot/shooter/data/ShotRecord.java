package frc.robot.shooter.data;

import edu.wpi.first.math.geometry.Pose2d;

public class ShotRecord {

    // NEW: unique shot ID
    public int id;

    public final double shooterRPM;
    public final double shooterVoltage;
    public final double statorCurrent;
    public final double supplyCurrent;
    public final double closedLoopError;

    public final Pose2d robotPose;
    public final double distanceToHub;

    public boolean made;
    public double missAmount;

    // Temporal metadata
    public double pressDuration;
    public int shotsInBurst;

    public ShotRecord(
        double rpm,
        double voltage,
        double statorCurrent,
        double supplyCurrent,
        double closedLoopError,
        Pose2d pose,
        double distanceToHub
    ) {
        this.shooterRPM = rpm;
        this.shooterVoltage = voltage;
        this.statorCurrent = statorCurrent;
        this.supplyCurrent = supplyCurrent;
        this.closedLoopError = closedLoopError;

        this.robotPose = pose;
        this.distanceToHub = distanceToHub;
    }

    public void setLabel(boolean made, double missAmount) {
        this.made = made;
        this.missAmount = missAmount;
    }

    public void setPressData(double pressDuration, int shotsInBurst) {
        this.pressDuration = pressDuration;
        this.shotsInBurst = shotsInBurst;
    }
}
