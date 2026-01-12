package frc.robot.shooter.model;

import frc.robot.shooter.data.ShotRecord;

public interface ShooterModel {
    double getRPM(ShotRecord telemetry);
}
