package frc.robot.shooter.model;

import frc.robot.shooter.data.PoseSupplier;
import frc.robot.shooter.data.ShotRecord;

public class RegressionShooterModel implements ShooterModel {

    private final double a; // distance
    private final double b; // stator current
    private final double c; // supply current
    private final double d; // closed-loop error
    private final double e; // pressDuration
    private final double f; // shotsInBurst
    private final double g; // bias

    private final PoseSupplier poseSupplier;

    public RegressionShooterModel(ModelConfig config, PoseSupplier poseSupplier) {
        this.a = config.a;
        this.b = config.b;
        this.c = config.c;
        this.d = config.d;
        this.e = config.e;
        this.f = config.f;
        this.g = config.g;

        this.poseSupplier = poseSupplier;
    }

    @Override
    public double getRPM(ShotRecord telemetry) {

        // Compute distance from robot pose to hub
        double distance = telemetry.distanceToTarget;

        // Extract telemetry
        double stator = telemetry.statorCurrent;
        double supply = telemetry.supplyCurrent;
        double error = telemetry.closedLoopError;

        // temporal features
        double pressDuration = telemetry.pressDuration;
        int shotsInBurst = telemetry.shotsInBurst;

        // Linear regression equation
        return
            a * distance +
            b * stator +
            c * supply +
            d * error +
            e * pressDuration +
            f * shotsInBurst +
            g;
    }
}
