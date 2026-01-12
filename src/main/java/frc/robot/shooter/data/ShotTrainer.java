package frc.robot.shooter.data;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.FieldElements.*;

public class ShotTrainer {

    private final TalonFX shooterMotor;
    private final PoseSupplier poseSupplier;

    private int nextShotId = 0;

    // Multiple pending shots
    private final List<ShotRecord> pendingShots = new ArrayList<>();

    // Root NT table: /Shooter/UnlabeledShots
    private final NetworkTable rootTable =
        NetworkTableInstance.getDefault()
            .getTable("Shooter")
            .getSubTable("UnlabeledShots");

    public ShotTrainer(TalonFX shooterMotor, PoseSupplier poseSupplier) {
        this.shooterMotor = shooterMotor;
        this.poseSupplier = poseSupplier;
    }

    public void notifyShotFired(double timestampSeconds) {
        recordShot(timestampSeconds);
    }

    private void recordShot(double timestampSeconds) {

        Pose2d pose = poseSupplier.getPose();
        double distance = pose.getTranslation().getDistance(HUB_POSE);

        double rpm = shooterMotor.getClosedLoopReference().getValueAsDouble();

        ShotRecord record = new ShotRecord(
            rpm,
            shooterMotor.getMotorVoltage().getValueAsDouble(),
            shooterMotor.getStatorCurrent().getValueAsDouble(),
            shooterMotor.getSupplyCurrent().getValueAsDouble(),
            shooterMotor.getClosedLoopError().getValueAsDouble(),
            pose,
            distance
        );

        record.id = nextShotId++;
        pendingShots.add(record);

        publishShot(record);
    }

    private void publishShot(ShotRecord r) {

        NetworkTable t = rootTable.getSubTable(Integer.toString(r.id));

        t.getEntry("id").setDouble(r.id);
        t.getEntry("distance").setDouble(r.distanceToHub);
        t.getEntry("rpm").setDouble(r.shooterRPM);
        t.getEntry("stator").setDouble(r.statorCurrent);
        t.getEntry("supply").setDouble(r.supplyCurrent);
        t.getEntry("error").setDouble(r.closedLoopError);

        // Elastic dropdown
        t.getEntry("label").setString("unlabeled");
    }

    public void update() {
        checkForLabels();
    }

    private void checkForLabels() {

        Iterator<ShotRecord> it = pendingShots.iterator();

        while (it.hasNext()) {
            ShotRecord r = it.next();

            NetworkTable t = rootTable.getSubTable(Integer.toString(r.id));
            String label = t.getEntry("label").getString("unlabeled");

            if (label == null || label.isBlank() ||
                label.equals("unlabeled") || label.equals("processed")) {
                continue;
            }

            // Parse real label
            ShotLabel parsed = ShotLabelParser.parse(label);
            r.setLabel(parsed.made(), parsed.missAmount());

            CSVWriter.writeSingleRecord(r);

            // Mark processed
            t.getEntry("label").setString("processed");

            // Clear NT subtable
            for (String key : t.getKeys()) {
                t.getEntry(key).unpublish();
            }

            // Unpublish the subtable key from the parent table
            rootTable.getEntry(Integer.toString(r.id)).unpublish();

            it.remove();
        }
    }
}
