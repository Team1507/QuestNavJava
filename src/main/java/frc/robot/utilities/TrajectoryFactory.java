package frc.robot.utilities;

// WPI libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.util.List;

public class TrajectoryFactory {
    public static Trajectory generateTrajectory(Pose2d start, Pose2d end) {
        // Configure max velocity and acceleration (tune for your robot)
        TrajectoryConfig config = new TrajectoryConfig(2.0, 2.0);

        // Generate a simple trajectory from start → end
        return TrajectoryGenerator.generateTrajectory(
            start,
            List.of(), // optional waypoints for obstacle avoidance
            end,
            config
        );
    }
}
