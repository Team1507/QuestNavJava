package frc.robot.utilities;

// WPI libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.Arrays;
import java.util.List;

// Robot Constants
import static frc.robot.Constants.FieldElements.*;
import static frc.robot.Constants.RobotGeometry.*;

public class TrajectoryFactory {

    /**
     * Generate a trajectory from start → waypoints → end.
     *
     * - First builds a straight-line trajectory.
     * - If the line between start and end intersects the reef polygon,
     *   detour waypoints are inserted to route around the reef.
     * - Detour side is chosen based on robot’s X position relative to reef center.
     * - After generation, the trajectory is checked for clearance against reef footprint.
     */
    public static Trajectory generateTrajectory(Pose2d start, Pose2d end) {
        // Configure trajectory limits (max velocity and acceleration).
        // These should be tuned for your robot’s drivetrain.
        TrajectoryConfig config = new TrajectoryConfig(2.0, 2.0);

        // Default: no intermediate waypoints (straight line).
        List<Translation2d> waypoints = List.of();
    
        // If the straight line intersects the reef polygon, insert detour waypoints.
        if (intersectsLine(start, end, REEF_HEX)) {
            // Choose detour side based on whether robot is left or right of reef center.
            if (start.getX() <= REEF_CENTER.getX()) {
                // Route around the left side of reef.
                waypoints = List.of(
                    new Translation2d(REEF_CENTER.getX() - CLEARANCE_X, REEF_CENTER.getY() + CLEARANCE_Y),
                    new Translation2d(REEF_CENTER.getX() - CLEARANCE_X, REEF_CENTER.getY() - CLEARANCE_Y)
                );
            } else {
                // Route around the right side of reef.
                waypoints = List.of(
                    new Translation2d(REEF_CENTER.getX() + CLEARANCE_X, REEF_CENTER.getY() + CLEARANCE_Y),
                    new Translation2d(REEF_CENTER.getX() + CLEARANCE_X, REEF_CENTER.getY() - CLEARANCE_Y)
                );
            }
        }
    
        // Generate trajectory through start → waypoints → end.
        Trajectory traj = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    
        // Unified clearance check: verifies robot footprint doesn’t overlap reef polygon.
        if (violatesClearance(traj, REEF_HEX)) {
            System.out.println("Trajectory collides with reef! Try regenerating with alternate detour side.");
        }
    
        return traj;
    }    

    /**
     * Unified clearance check.
     *
     * - Samples each state along the trajectory.
     * - Computes robot’s polygon footprint at that pose.
     * - Tests overlap between robot polygon and reef polygon.
     * - Returns true if any collision is detected.
     */
    public static boolean violatesClearance(Trajectory traj, List<Translation2d> reefHex) {
        for (Trajectory.State s : traj.getStates()) {
            List<Translation2d> robotPoly = robotCorners(s.poseMeters);
            if (polygonsOverlap(robotPoly, reefHex)) return true;
        }
        return false;
    }

    /**
     * Compute robot corners (polygon footprint) at a given pose.
     *
     * - Uses HALF_LENGTH_METERS and HALF_WIDTH_METERS from Constants.RobotGeometry.
     * - Applies rotation to offsets so corners rotate correctly with robot heading.
     * - Returns a list of four Translation2d points representing the chassis corners.
     */
    public static List<Translation2d> robotCorners(Pose2d pose) {
        Rotation2d rot = pose.getRotation();
        Translation2d center = pose.getTranslation();

        // Define offsets from robot center to each corner.
        Translation2d[] offsets = {
            new Translation2d(HALF_LENGTH_METERS,  HALF_WIDTH_METERS),
            new Translation2d(HALF_LENGTH_METERS, -HALF_WIDTH_METERS),
            new Translation2d(-HALF_LENGTH_METERS, -HALF_WIDTH_METERS),
            new Translation2d(-HALF_LENGTH_METERS,  HALF_WIDTH_METERS)
        };

        // Rotate offsets by robot heading and translate to robot center.
        return Arrays.stream(offsets)
            .map(o -> new Translation2d(
                center.getX() + o.getX() * rot.getCos() - o.getY() * rot.getSin(),
                center.getY() + o.getX() * rot.getSin() + o.getY() * rot.getCos()
            ))
            .toList();
    }

    /**
     * Polygon overlap check.
     *
     * - First checks if any edges of polyA intersect edges of polyB.
     * - Then checks containment: whether any corner of one polygon lies inside the other.
     * - Returns true if overlap is detected.
     */
    private static boolean polygonsOverlap(List<Translation2d> polyA, List<Translation2d> polyB) {
        // Edge intersection check.
        for (int i = 0; i < polyA.size(); i++) {
            Translation2d a1 = polyA.get(i);
            Translation2d a2 = polyA.get((i + 1) % polyA.size());
            for (int j = 0; j < polyB.size(); j++) {
                Translation2d b1 = polyB.get(j);
                Translation2d b2 = polyB.get((j + 1) % polyB.size());
                if (lineIntersects(a1, a2, b1, b2)) return true;
            }
        }
        // Containment check: corners inside other polygon.
        for (Translation2d corner : polyA) {
            if (pointInPolygon(corner, polyB)) return true;
        }
        for (Translation2d corner : polyB) {
            if (pointInPolygon(corner, polyA)) return true;
        }
        return false;
    }

    /**
     * Utility: check if two line segments intersect.
     *
     * - Uses orientation tests to determine if segments straddle each other.
     * - Returns true if intersection occurs.
     */
    private static boolean lineIntersects(Translation2d p1, Translation2d p2,
                                          Translation2d q1, Translation2d q2) {
        double d1 = direction(q1, q2, p1);
        double d2 = direction(q1, q2, p2);
        double d3 = direction(p1, p2, q1);
        double d4 = direction(p1, p2, q2);

        return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
                ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)));
    }

    /**
     * Point-in-polygon check (ray casting algorithm).
     *
     * - Casts a horizontal ray from point p.
     * - Counts how many times it intersects polygon edges.
     * - Odd count = inside, even count = outside.
     */
    private static boolean pointInPolygon(Translation2d p, List<Translation2d> poly) {
        boolean inside = false;
        for (int i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
            double xi = poly.get(i).getX(), yi = poly.get(i).getY();
            double xj = poly.get(j).getX(), yj = poly.get(j).getY();
            boolean intersect = ((yi > p.getY()) != (yj > p.getY())) &&
                                (p.getX() < (xj - xi) * (p.getY() - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    /**
     * Helper: orientation test.
     *
     * - Computes signed area of triangle (a, b, c).
     * - Positive/negative result indicates relative orientation.
     * - Used by lineIntersects to detect straddling.
     */
    private static double direction(Translation2d a, Translation2d b, Translation2d c) {
        return (c.getX() - a.getX()) * (b.getY() - a.getY()) -
               (b.getX() - a.getX()) * (c.getY() - a.getY());
    }

    /**
     * Quick line intersection check for start→end vs reef polygon.
     *
     * - Converts start/end poses to Translation2d.
     * - Checks if line segment intersects any reef polygon edge.
     * - Used to decide whether detour waypoints are needed.
     */
    private static boolean intersectsLine(Pose2d start, Pose2d end, List<Translation2d> polygon) {
        Translation2d p1 = start.getTranslation();
        Translation2d p2 = end.getTranslation();
        for (int i = 0; i < polygon.size(); i++) {
            Translation2d q1 = polygon.get(i);
            Translation2d q2 = polygon.get((i + 1) % polygon.size());
            if (lineIntersects(p1, p2, q1, q2)) return true;
        }
        return false;
    }
}
