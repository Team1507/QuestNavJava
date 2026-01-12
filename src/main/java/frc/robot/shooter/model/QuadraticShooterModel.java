package frc.robot.shooter.model;

import frc.robot.shooter.data.ShotRecord;

public class QuadraticShooterModel implements ShooterModel {

    private final double a; // linear distance
    private final double b; // quadratic distance^2
    private final double g; // bias

    public QuadraticShooterModel(ModelConfig config) {
        this.a = config.a;
        this.b = config.b;
        this.g = config.g;
    }

    @Override
    public double getRPM(ShotRecord telemetry) {

        double d = telemetry.distanceToHub;

        // Quadratic model:
        // rpm = b*d^2 + a*d + g
        return (b * d * d) + (a * d) + g;
    }
}
