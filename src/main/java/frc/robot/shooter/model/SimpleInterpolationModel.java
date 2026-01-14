package frc.robot.shooter.model;

import frc.robot.shooter.data.ShotRecord;

public class SimpleInterpolationModel implements ShooterModel {

    @Override
    public double getRPM(ShotRecord telemetry) {

        double distance = telemetry.distanceToTarget;

        // Simple linear fallback curve
        return 3000 + (distance * 200);
    }
}
