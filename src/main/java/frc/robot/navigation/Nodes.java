package frc.robot.navigation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Full competitive navigation node map for the 2026 REBUILT field.
 *
 * All coordinates are:
 *  - In meters
 *  - In WPILib field coordinates (origin at Blue DS corner)
 *  - Derived from official AprilTag locations and REBUILT field geometry
 *
 * Approach nodes:
 *  - 1.0 m away from the structure
 *  - Facing directly toward the structure center
 */
public final class Nodes {

    private static final double APPROACH_DISTANCE_METERS = 1.0;

    private Nodes() {}

    private static Pose2d approach(Pose2d from, Pose2d to) {
        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();
        double angle = Math.atan2(dy, dx);
        double ax = from.getX() - APPROACH_DISTANCE_METERS * Math.cos(angle);
        double ay = from.getY() - APPROACH_DISTANCE_METERS * Math.sin(angle);
        return new Pose2d(ax, ay, new Rotation2d(angle));
    }

    // ============================
    // HUB NODES (Blue-side reference)
    // ============================
    public static final class Hub {
        // Center of the Hub (computed from all 8 tags)
        public static final Pose2d CENTER =
                new Pose2d(4.545, 4.020, Rotation2d.fromDegrees(0.0));

        // Faces (approximate, based on tag clusters)
        public static final Pose2d FRONT_LEFT =
                new Pose2d(4.612, 4.626, Rotation2d.fromDegrees(90.0));
        public static final Pose2d FRONT_RIGHT =
                new Pose2d(4.256, 4.626, Rotation2d.fromDegrees(90.0));
        public static final Pose2d BACK_LEFT =
                new Pose2d(4.612, 3.417, Rotation2d.fromDegrees(270.0));
        public static final Pose2d BACK_RIGHT =
                new Pose2d(4.256, 3.417, Rotation2d.fromDegrees(270.0));

        // Approach nodes (1.0 m away, facing center)
        public static final Pose2d APPROACH_FRONT =
                approach(FRONT_LEFT, CENTER);
        public static final Pose2d APPROACH_BACK =
                approach(BACK_LEFT, CENTER);
        public static final Pose2d APPROACH_LEFT =
                approach(FRONT_RIGHT, CENTER);
        public static final Pose2d APPROACH_RIGHT =
                approach(BACK_RIGHT, CENTER);
    }

    // ============================
    // TOWER NODES (Blue-side reference)
    // ============================
    public static final class Tower {
        // Tower center from tags at (0.55 in, 146.86/163.86 in)
        public static final Pose2d CENTER =
                new Pose2d(0.014, 3.947, Rotation2d.fromDegrees(0.0));

        // Virtual face points around the tower (for approach geometry)
        private static final Pose2d FRONT_POINT =
                new Pose2d(CENTER.getX() + 0.50, CENTER.getY(), Rotation2d.fromDegrees(0.0));
        private static final Pose2d BACK_POINT =
                new Pose2d(CENTER.getX() - 0.50, CENTER.getY(), Rotation2d.fromDegrees(180.0));
        private static final Pose2d LEFT_POINT =
                new Pose2d(CENTER.getX(), CENTER.getY() + 0.50, Rotation2d.fromDegrees(90.0));
        private static final Pose2d RIGHT_POINT =
                new Pose2d(CENTER.getX(), CENTER.getY() - 0.50, Rotation2d.fromDegrees(270.0));

        public static final Pose2d APPROACH_FRONT =
                approach(FRONT_POINT, CENTER);
        public static final Pose2d APPROACH_BACK =
                approach(BACK_POINT, CENTER);
        public static final Pose2d APPROACH_LEFT =
                approach(LEFT_POINT, CENTER);
        public static final Pose2d APPROACH_RIGHT =
                approach(RIGHT_POINT, CENTER);
    }

    // ============================
    // DEPOT NODES (Blue-side reference)
    // ============================
    public static final class Depot {
        // Depot center from tags at (0.54 in, 25.62/42.62 in)
        public static final Pose2d CENTER =
                new Pose2d(0.014, 0.867, Rotation2d.fromDegrees(0.0));

        private static final Pose2d LEFT_POINT =
                new Pose2d(CENTER.getX(), CENTER.getY() + 0.50, Rotation2d.fromDegrees(90.0));
        private static final Pose2d RIGHT_POINT =
                new Pose2d(CENTER.getX(), CENTER.getY() - 0.50, Rotation2d.fromDegrees(270.0));

        public static final Pose2d ENTRY_LEFT =
                approach(LEFT_POINT, CENTER);
        public static final Pose2d ENTRY_RIGHT =
                approach(RIGHT_POINT, CENTER);

        public static final Pose2d PICKUP_LEFT =
                approach(LEFT_POINT, CENTER);
        public static final Pose2d PICKUP_RIGHT =
                approach(RIGHT_POINT, CENTER);
    }

    // ============================
    // OUTPOST NODES (Blue-side reference)
    // ============================
    public static final class Outpost {
        // From tags at (0.54 in, 25.62 in) and (0.54 in, 42.62 in)
        public static final Pose2d LEFT =
                new Pose2d(0.014, 0.651, Rotation2d.fromDegrees(0.0));
        public static final Pose2d RIGHT =
                new Pose2d(0.014, 1.082, Rotation2d.fromDegrees(0.0));

        private static final Pose2d LEFT_APPROACH_POINT =
                new Pose2d(0.60, 0.651, Rotation2d.fromDegrees(0.0));
        private static final Pose2d RIGHT_APPROACH_POINT =
                new Pose2d(0.60, 1.082, Rotation2d.fromDegrees(0.0));

        public static final Pose2d APPROACH_LEFT =
                approach(LEFT_APPROACH_POINT, LEFT);
        public static final Pose2d APPROACH_RIGHT =
                approach(RIGHT_APPROACH_POINT, RIGHT);
    }

    // ============================
    // TRENCH NODES (Blue-side reference)
    // ============================
    public static final class Trench {
        // From tags at (183.03 in, 24.85 in) and (183.03 in, 291.79 in)
        public static final Pose2d NEAR =
                new Pose2d(4.649, 0.631, Rotation2d.fromDegrees(0.0));
        public static final Pose2d FAR =
                new Pose2d(4.649, 7.409, Rotation2d.fromDegrees(0.0));

        public static final Pose2d ENTRY =
                new Pose2d(3.810, 0.631, Rotation2d.fromDegrees(0.0));
        public static final Pose2d EXIT =
                new Pose2d(3.810, 7.409, Rotation2d.fromDegrees(0.0));
    }

    // ============================
    // MIDFIELD NODES
    // ============================
    public static final class Midfield {
        // Using REBUILT carpet edges (B2) – approximate center and safe lanes
        // These values assume a field roughly 16.46 m long by ~8.23 m wide.
        public static final Pose2d CENTER =
                new Pose2d(8.230, 4.115, Rotation2d.fromDegrees(0.0));

        public static final Pose2d SAFE_LEFT =
                new Pose2d(8.230, 5.000, Rotation2d.fromDegrees(0.0));
        public static final Pose2d SAFE_RIGHT =
                new Pose2d(8.230, 3.200, Rotation2d.fromDegrees(0.0));

        public static final Pose2d TOWER_BYPASS_LEFT =
                new Pose2d(1.000, 5.000, Rotation2d.fromDegrees(0.0));
        public static final Pose2d TOWER_BYPASS_RIGHT =
                new Pose2d(1.000, 3.200, Rotation2d.fromDegrees(0.0));
    }

    // ============================
    // STARTING POSITIONS (Blue-side)
    // ============================
    public static final class Start {
        // 2.0 m from Blue wall, spread across width
        public static final Pose2d LEFT =
                new Pose2d(2.000, 2.000, Rotation2d.fromDegrees(0.0));
        public static final Pose2d CENTER =
                new Pose2d(2.000, 4.115, Rotation2d.fromDegrees(0.0));
        public static final Pose2d RIGHT =
                new Pose2d(2.000, 6.200, Rotation2d.fromDegrees(0.0));
    }

    // ============================
    // PARKING / ENDGAME (Red-side)
    // ============================
    public static final class Park {
        // 2.0 m from Red wall, mirrored in X from Blue side
        // Assuming field length ≈ 16.46 m
        private static final double FIELD_LENGTH_METERS = 16.460;
        private static final double PARK_X = FIELD_LENGTH_METERS - 2.000;

        public static final Pose2d LEFT =
                new Pose2d(PARK_X, 2.000, Rotation2d.fromDegrees(180.0));
        public static final Pose2d CENTER =
                new Pose2d(PARK_X, 4.115, Rotation2d.fromDegrees(180.0));
        public static final Pose2d RIGHT =
                new Pose2d(PARK_X, 6.200, Rotation2d.fromDegrees(180.0));
    }
}
