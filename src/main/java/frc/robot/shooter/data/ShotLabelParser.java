package frc.robot.shooter.data;

public class ShotLabelParser {

    public static ShotLabel parse(String label) {

        switch (label.toLowerCase()) {

            case "went in":
                return new ShotLabel(true, 0.0);

            case "went in but was short":
                return new ShotLabel(true, -0.2);

            case "went in but was far":
                return new ShotLabel(true, 0.2);

            case "barely missed short":
                return new ShotLabel(false, -0.3);

            case "barely missed far":
                return new ShotLabel(false, 0.3);

            case "way too short":
                return new ShotLabel(false, -1.0);

            case "way too far":
                return new ShotLabel(false, 1.0);

            case "unlabeled":
            default:
                return new ShotLabel(false, 0.0);
        }
    }
}
