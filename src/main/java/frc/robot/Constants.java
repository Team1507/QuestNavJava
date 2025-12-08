// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class RobotGeometry {
        public static final double HALF_LENGTH_METERS = 0.35; // half of robot length
        public static final double HALF_WIDTH_METERS  = 0.35; // half of robot width
    }
    
    public static final class IO {
        public static final int JOYSTICK_PORT = 0;
    }

    public static final class Drive {
        public static final double TRANSLATION_SCALE    = 0.9;  // 0.15 // Input from X and Y controller to limit max speed from controller
        public static final double ROTATION_SCALE       = 0.9;  // 0.25 // Input from X of controller to limit max speed from controller
    }

    public static final class Quest {
        public static final Transform2d ROBOT_TO_QUEST_TRANSFORM =
            new Transform2d(
                Units.inchesToMeters(-2.274634152842048),
                Units.inchesToMeters(7.481121673003204),
                Rotation2d.kCCW_90deg);

        public static final Matrix<N3, N1> STD_DEVS = VecBuilder.fill(0, 0, 0);

        public static final double ACCEPTABLE_DISTANCE_TOLERANCE = Units.inchesToMeters(4);
        
        // Mount transform: robot origin -> QuestNav sensor
        public static final Transform3d ROBOT_TO_QUEST =
            new Transform3d(0.381, 0.0, 0.3048, new Rotation3d(0.0, 0.0, 0.0));

        // Standard deviations for measurement trust
        public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
            0.02,  // 2 cm X
            0.02,  // 2 cm Y
            0.035  // ~2 degrees
        );
    }

    public static final class MoveToPose {
        // --- PID gains for X, Y, and rotation ---
        public static final double X_KP     = 1.0;
        public static final double X_KI     = 0.0;
        public static final double X_KD     = 0.0;

        public static final double Y_KP     = 1.0;
        public static final double Y_KI     = 0.0;
        public static final double Y_KD     = 0.0;

        public static final double THETA_KP = 2.0;
        public static final double THETA_KI = 0.0;
        public static final double THETA_KD = 0.0;

        // --- Speed cap and deadband ---
        public static final double MAX_LINEAR_SPEED     = 2.0;  // m/s cap for testing
        public static final double MAX_ANGULAR_SPEED    = 3.0;  // rad/s cap for testing
        public static final double DEADBAND_ERROR       = 0.02; // meters, near target

        // --- Tolerances ---
        public static final double POSITION_TOLERANCE_METERS    = 0.05;
        public static final double ANGLE_TOLERANCE_RADIANS      = Math.toRadians(2.0);

        // --- Timeouts ---
        public static final double STALL_THRESHOLD  = 0.02; // meters
        public static final double STALL_TIMEOUT    = 1.0;  // seconds
        
        // --- Poses ---
        public static final Pose2d POSE_A = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(90));
        public static final Pose2d POSE_B = new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(180));
    }

    public static final class FieldElements {
        // --- Reef ---
        public static final Translation2d REEF_CENTER = new Translation2d(4.5, 4.0);
        public static final double CLEARANCE_Y = 1.5; // vertical margin
        public static final double CLEARANCE_X = 1.5; // lateral margin
        public static final List<Translation2d> REEF_HEX = List.of(
            new Translation2d(4.5, 5.0),
            new Translation2d(3.65, 4.4),
            new Translation2d(3.65, 3.5),
            new Translation2d(4.5, 3.0),
            new Translation2d(5.35, 3.5),
            new Translation2d(5.35, 4.4)
        );
    }
}
