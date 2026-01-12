// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

// WPI libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.*;

// Robot Constants
import static frc.robot.Constants.FieldElements.*;
import static frc.robot.Constants.RobotGeometry.*;

public class RRTGenerator {

    /**
     * Generate a trajectory using RRT → RRT* with clearance checks.
     *
     * - Builds a tree of random samples within the field boundary.
     * - Rejects samples inside or near the reef polygon (inflated by robot footprint).
     * - Connects samples into a tree until the goal is reached.
     * - Uses RRT* rewiring to optimize path cost.
     * - Converts the final waypoint sequence into a WPILib trajectory.
     */
    public static Trajectory generateTrajectoryRRT(Pose2d start, Pose2d end) {
        // Configure trajectory limits (max velocity and acceleration).
        TrajectoryConfig config = new TrajectoryConfig(2.0, 2.0);

        // Start timing for performance measurement.
        long t0 = System.nanoTime();

        // Inflate reef polygon by half the robot diagonal.
        // This ensures clearance: if the robot center avoids the inflated reef,
        // the actual robot footprint avoids the real reef.
        double inflate = Math.hypot(HALF_LENGTH_METERS, HALF_WIDTH_METERS) + 0.05;
        List<Translation2d> inflatedReef = inflatePolygon(REEF_HEX, inflate);

        // Initialize RRT tree with root node at the start position.
        RRTNode root = new RRTNode(start.getTranslation(), null, 0.0);
        List<RRTNode> tree = new ArrayList<>();
        tree.add(root);

        Random rand = new Random();
        RRTNode goalNode = null; // will hold the final goal node once reached

        // Algorithm parameters (tune for reliability vs speed).
        int maxSamples = 600;       // number of random samples to attempt
        double stepSize = 0.4;      // expansion step size (meters)
        double neighborRadius = 0.8;// radius for RRT* rewiring
        double goalSnapRadius = 0.6;// distance threshold to consider goal "reached"

        // -------------------------------
        // RRT Exploration Phase
        // -------------------------------
        for (int i = 0; i < maxSamples; i++) {
            // Generate a random valid sample point inside the field and outside reef.
            Translation2d sample = randomValidSample(rand);

            // Find nearest existing node in the tree to this sample.
            RRTNode nearest = findNearest(tree, sample);

            // Steer from nearest node toward the sample by stepSize.
            Translation2d newPoint = steer(nearest.point, sample, stepSize);

            // Reject if new point is outside field or collides with reef footprint.
            if (!insideField(newPoint) || violatesClearance(new Pose2d(newPoint, new Rotation2d()), inflatedReef)) continue;

            // Reject if the edge (nearest → newPoint) intersects reef footprint.
            if (edgeCollides(nearest.point, newPoint, inflatedReef)) continue;

            // Add new node to the tree.
            RRTNode newNode = new RRTNode(newPoint, nearest, nearest.cost + nearest.point.getDistance(newPoint));
            tree.add(newNode);

            // Check if new node is close enough to the goal.
            if (newPoint.getDistance(end.getTranslation()) < goalSnapRadius) {
                // Ensure final edge (newPoint → goal) is collision-free.
                if (!edgeCollides(newPoint, end.getTranslation(), inflatedReef)) {
                    goalNode = new RRTNode(end.getTranslation(), newNode,
                            newNode.cost + newPoint.getDistance(end.getTranslation()));
                    tree.add(goalNode);
                    break; // stop exploration once goal is reached
                }
            }
        }

        // -------------------------------
        // Fallback: connect nearest node to goal if exploration failed
        // -------------------------------
        if (goalNode == null) {
            RRTNode nearestToGoal = findNearest(tree, end.getTranslation());
            if (!edgeCollides(nearestToGoal.point, end.getTranslation(), inflatedReef)) {
                goalNode = new RRTNode(end.getTranslation(), nearestToGoal,
                        nearestToGoal.cost + nearestToGoal.point.getDistance(end.getTranslation()));
                tree.add(goalNode);
            }
        }

        // If still no goal, bail safely with straight-line trajectory.
        if (goalNode == null) {
            System.out.println("[RRT] No path found. Falling back to straight-line trajectory.");
            return TrajectoryGenerator.generateTrajectory(start, List.of(), end, config);
        }

        // -------------------------------
        // RRT* Rewiring Phase
        // -------------------------------
        // Try to improve path cost by reconnecting nearby nodes if cheaper.
        for (RRTNode node : tree) {
            for (RRTNode neighbor : findNearby(tree, node, neighborRadius)) {
                double candidateCost = node.cost + node.point.getDistance(neighbor.point);
                if (candidateCost < neighbor.cost && !edgeCollides(node.point, neighbor.point, inflatedReef)) {
                    neighbor.parent = node;
                    neighbor.cost = candidateCost;
                }
            }
        }

        // -------------------------------
        // Path Extraction Phase
        // -------------------------------
        List<Translation2d> waypoints = new ArrayList<>();
        RRTNode current = goalNode;
        while (current != null) {
            waypoints.add(current.point);
            current = current.parent;
        }
        Collections.reverse(waypoints); // reverse to get start → goal order

        // Remove duplicate start/end points (TrajectoryGenerator takes them separately).
        if (!waypoints.isEmpty() && waypoints.get(0).getDistance(start.getTranslation()) < 1e-6) waypoints.remove(0);
        if (!waypoints.isEmpty() && waypoints.get(waypoints.size() - 1).getDistance(end.getTranslation()) < 1e-6) waypoints.remove(waypoints.size() - 1);

        // If path extraction failed, fall back safely.
        if (waypoints.isEmpty()) {
            System.out.println("[RRT] Empty waypoint path after extraction. Using straight-line trajectory.");
            return TrajectoryGenerator.generateTrajectory(start, List.of(), end, config);
        }

        // -------------------------------
        // Final Trajectory Generation
        // -------------------------------
        Trajectory traj = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);

        // Print debug info: number of nodes, waypoints, and runtime.
        long t1 = System.nanoTime();
        System.out.printf("[RRT] nodes=%d, waypoints=%d, time=%.2f ms%n",
                tree.size(), waypoints.size(), (t1 - t0) / 1e6);

        return traj;
    }

    // ##################################
    // ######### Helper Classes #########
    // ##################################

    /**
     * Represents a node in the RRT tree.
     * Each node stores:
     * - point: the 2D position in the field (Translation2d).
     * - parent: the node that this one was connected from (used for backtracking the path).
     * - cost: cumulative path cost (distance from the start node to this node).
     */
    private static class RRTNode {
        Translation2d point;
        RRTNode parent;
        double cost;

        /**
         * Constructs a new RRTNode.
         *
         * @param point  The 2D position of this node in the field.
         * @param parent The parent node that connects this node to the tree.
         * @param cost   The cumulative cost (distance) from the start node to this node.
         */
        RRTNode(Translation2d point, RRTNode parent, double cost) {
            this.point = point;
            this.parent = parent;
            this.cost = cost;
        }
    }

    // ##################################
    // ######### Helper Methods #########
    // ##################################

    /**
     * Generate a random sample point inside the field boundaries.
     * This is used during RRT exploration to expand the tree toward random directions.
     *
     * @param rand Random number generator.
     * @return A random Translation2d point within the field dimensions.
     */
    private static Translation2d randomSample(Random rand) {
        double x = rand.nextDouble() * FIELD_LENGTH;
        double y = rand.nextDouble() * FIELD_WIDTH;
        return new Translation2d(x, y);
    }

    /**
     * Find the nearest node in the tree to a given sample point.
     * This is the "nearest neighbor" step of RRT, which determines
     * where the tree should expand toward the sample.
     *
     * @param tree   The list of nodes currently in the RRT tree.
     * @param sample The random sample point to compare against.
     * @return The nearest RRTNode in the tree to the sample point.
     */
    private static RRTNode findNearest(List<RRTNode> tree, Translation2d sample) {
        RRTNode nearest = tree.get(0);
        double minDist = nearest.point.getDistance(sample);
        for (RRTNode node : tree) {
            double dist = node.point.getDistance(sample);
            if (dist < minDist) {
                minDist = dist;
                nearest = node;
            }
        }
        return nearest;
    }

    /**
     * Steer from one point toward another by a fixed step size.
     * If the target point is closer than stepSize, return the target directly.
     * Otherwise, move stepSize units along the line toward the target.
     *
     * @param from     Starting point (Translation2d).
     * @param to       Target point (Translation2d).
     * @param stepSize Maximum distance to move toward the target.
     * @return A new Translation2d point that is stepSize closer to the target.
     */
    private static Translation2d steer(Translation2d from, Translation2d to, double stepSize) {
        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();
        double dist = Math.sqrt(dx*dx + dy*dy);
        if (dist < stepSize) return to; // target is within step size
        double scale = stepSize / dist;
        return new Translation2d(from.getX() + dx * scale, from.getY() + dy * scale);
    }

    /**
     * Check if a point collides with the reef polygon.
     * This is a simple point-in-polygon test (not footprint-aware).
     *
     * @param point   The point to test.
     * @param reefHex The reef polygon defined as a list of Translation2d vertices.
     * @return True if the point lies inside the reef polygon, false otherwise.
     */
    private static boolean collides(Translation2d point, List<Translation2d> reefHex) {
        return pointInPolygon(point, reefHex);
    }

    /**
     * Find all nodes in the tree that are within a given radius of a node.
     * This is used in RRT* for rewiring: nearby nodes may be reconnected
     * if a cheaper path through the current node is found.
     *
     * @param tree   The list of nodes currently in the RRT tree.
     * @param node   The node to compare distances against.
     * @param radius The maximum distance to consider a node "nearby".
     * @return A list of RRTNodes within the given radius of the node.
     */
    private static List<RRTNode> findNearby(List<RRTNode> tree, RRTNode node, double radius) {
        List<RRTNode> neighbors = new ArrayList<>();
        for (RRTNode other : tree) {
            if (other != node && other.point.getDistance(node.point) < radius) {
                neighbors.add(other);
            }
        }
        return neighbors;
    }

    /**
     * Check if the edge (line segment) between two points collides with the reef polygon.
     * - Samples intermediate points along the edge.
     * - At each sample, builds a Pose2d and checks robot footprint clearance.
     *
     * @param a        Starting point of the edge.
     * @param b        Ending point of the edge.
     * @param reefPoly Inflated reef polygon to test against.
     * @return True if any sampled point along the edge collides with reef, false otherwise.
     */
    private static boolean edgeCollides(Translation2d a, Translation2d b, List<Translation2d> reefPoly) {
        double edgeLength = a.getDistance(b);
        int steps = Math.max(10, (int)(edgeLength / 0.1)); // sample every 10cm
        for (int i = 0; i <= steps; i++) {
            double t = i / (double) steps;
            double x = a.getX() + t * (b.getX() - a.getX());
            double y = a.getY() + t * (b.getY() - a.getY());
            Pose2d pose = new Pose2d(x, y, new Rotation2d());
            if (violatesClearance(pose, reefPoly)) return true;
        }
        return false;
    }

    /**
     * Generate a random sample point that is valid (inside field and outside reef).
     * - Retries up to 100 times to find a valid free-space point.
     * - Falls back to field center if no valid sample is found.
     *
     * @param rand Random number generator.
     * @return A valid Translation2d sample point.
     */
    private static Translation2d randomValidSample(Random rand) {
        for (int attempts = 0; attempts < 100; attempts++) {
            double x = rand.nextDouble() * FIELD_LENGTH;
            double y = rand.nextDouble() * FIELD_WIDTH;
            Translation2d p = new Translation2d(x, y);
            if (insideField(p) && !insideOrNearReef(p)) {
                return p;
            }
        }
        // Fallback: center of field if sampling fails
        return new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);
    }

    /**
     * Check if a point lies inside the rectangular field boundary.
     *
     * @param p Point to test.
     * @return True if inside field, false otherwise.
     */
    private static boolean insideField(Translation2d p) {
        return p.getX() >= 0 && p.getX() <= FIELD_LENGTH &&
            p.getY() >= 0 && p.getY() <= FIELD_WIDTH;
    }

    /**
     * Check if a point lies inside or too close to the reef.
     * - Inflates reef polygon by half robot diagonal for safety margin.
     *
     * @param p Point to test.
     * @return True if inside or near reef, false otherwise.
     */
    private static boolean insideOrNearReef(Translation2d p) {
        double inflate = Math.hypot(HALF_LENGTH_METERS, HALF_WIDTH_METERS);
        return pointInPolygon(p, inflatePolygon(REEF_HEX, inflate));
    }

    /**
     * Check if a line segment intersects a polygon.
     * - Iterates over polygon edges and tests intersection.
     *
     * @param a       Start point of segment.
     * @param b       End point of segment.
     * @param polygon Polygon vertices.
     * @return True if segment intersects polygon, false otherwise.
     */
    private static boolean segmentHitsPolygon(Translation2d a, Translation2d b, List<Translation2d> polygon) {
        for (int i = 0; i < polygon.size(); i++) {
            Translation2d q1 = polygon.get(i);
            Translation2d q2 = polygon.get((i + 1) % polygon.size());
            if (lineIntersects(a, b, q1, q2)) return true;
        }
        return false;
    }

    /**
     * Inflate a polygon outward by a margin.
     * - Shifts each vertex away from centroid by 'margin' units.
     * - Works best for convex polygons like the reef hex.
     *
     * @param poly   Original polygon vertices.
     * @param margin Distance to inflate outward.
     * @return Inflated polygon vertices.
     */
    private static List<Translation2d> inflatePolygon(List<Translation2d> poly, double margin) {
        // Compute centroid
        double cx = 0, cy = 0;
        for (Translation2d v : poly) { cx += v.getX(); cy += v.getY(); }
        cx /= poly.size(); cy /= poly.size();

        List<Translation2d> inflated = new ArrayList<>(poly.size());
        for (Translation2d v : poly) {
            double dx = v.getX() - cx;
            double dy = v.getY() - cy;
            double len = Math.hypot(dx, dy);
            double scale = (len + margin) / (len == 0 ? 1.0 : len);
            inflated.add(new Translation2d(cx + dx * scale, cy + dy * scale));
        }
        return inflated;
    }

    /**
     * Check if the robot footprint at a given pose overlaps the reef polygon.
     *
     * @param pose     Robot pose (position + orientation).
     * @param reefPoly Inflated reef polygon.
     * @return True if robot footprint overlaps reef, false otherwise.
     */
    private static boolean violatesClearance(Pose2d pose, List<Translation2d> reefPoly) {
        List<Translation2d> robotPoly = robotCorners(pose);
        return polygonsOverlap(robotPoly, reefPoly);
    }

    /**
     * Compute the four corners of the robot chassis polygon at a given pose.
     * - Uses HALF_LENGTH_METERS and HALF_WIDTH_METERS constants for robot geometry.
     * - Applies rotation to offsets so corners rotate correctly with robot heading.
     *
     * @param pose Robot pose.
     * @return List of Translation2d points representing robot corners.
     */
    private static List<Translation2d> robotCorners(Pose2d pose) {
        Rotation2d rot = pose.getRotation();
        Translation2d center = pose.getTranslation();

        Translation2d[] offsets = {
            new Translation2d(HALF_LENGTH_METERS,  HALF_WIDTH_METERS),
            new Translation2d(HALF_LENGTH_METERS, -HALF_WIDTH_METERS),
            new Translation2d(-HALF_LENGTH_METERS, -HALF_WIDTH_METERS),
            new Translation2d(-HALF_LENGTH_METERS,  HALF_WIDTH_METERS)
        };

        return Arrays.stream(offsets)
            .map(o -> new Translation2d(
                center.getX() + o.getX() * rot.getCos() - o.getY() * rot.getSin(),
                center.getY() + o.getX() * rot.getSin() + o.getY() * rot.getCos()
            ))
            .toList();
    }

    /**
     * Check if two polygons overlap.
     * - Tests edge intersections.
     * - Tests containment of corners inside the other polygon.
     *
     * @param polyA First polygon vertices.
     * @param polyB Second polygon vertices.
     * @return True if polygons overlap, false otherwise.
     */
    private static boolean polygonsOverlap(List<Translation2d> polyA, List<Translation2d> polyB) {
        for (int i = 0; i < polyA.size(); i++) {
            Translation2d a1 = polyA.get(i);
            Translation2d a2 = polyA.get((i + 1) % polyA.size());
            for (int j = 0; j < polyB.size(); j++) {
                Translation2d b1 = polyB.get(j);
                Translation2d b2 = polyB.get((j + 1) % polyB.size());
                if (lineIntersects(a1, a2, b1, b2)) return true;
            }
        }
        for (Translation2d corner : polyA) {
            if (pointInPolygon(corner, polyB)) return true;
        }
        for (Translation2d corner : polyB) {
            if (pointInPolygon(corner, polyA)) return true;
        }
        return false;
    }

    /**
     * Check if two line segments intersect using orientation tests.
     *
     * @param p1 First endpoint of segment 1.
     * @param p2 Second endpoint of segment 1.
     * @param q1 First endpoint of segment 2.
     * @param q2 Second endpoint of segment 2.
     * @return True if segments intersect, false otherwise.
     */
    private static boolean lineIntersects(Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
        double d1 = direction(q1, q2, p1);
        double d2 = direction(q1, q2, p2);
        double d3 = direction(p1, p2, q1);
        double d4 = direction(p1, p2, q2);

        return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
                ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)));
    }

    /**
     * Point-in-polygon check using ray casting algorithm.
     * - Casts a horizontal ray from point p.
     * - Counts intersections with polygon edges.
     * - Odd count = inside, even count = outside.
     *
     * @param p    Point to test.
     * @param poly Polygon vertices.
     * @return True if point is inside polygon, false otherwise.
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
     * Compute the orientation (signed area) of the triangle formed by three points.
     *
     * - This function is used in line intersection tests.
     * - It calculates the cross product of vectors (a→c) and (a→b).
     * - The result indicates the relative orientation of point c with respect to the line segment ab:
     *      > 0 : c is to the left of the directed line from a to b
     *      < 0 : c is to the right of the directed line from a to b
     *      = 0 : points a, b, and c are collinear
     *
     * @param a First point (start of line segment).
     * @param b Second point (end of line segment).
     * @param c Third point to test orientation against line ab.
     * @return Signed value representing orientation:
     *         positive = left turn, negative = right turn, zero = collinear.
     */
    private static double direction(Translation2d a, Translation2d b, Translation2d c) {
        return (c.getX() - a.getX()) * (b.getY() - a.getY()) -
            (b.getX() - a.getX()) * (c.getY() - a.getY());
    }
}
