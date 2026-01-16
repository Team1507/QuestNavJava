// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // ============================================================
    //  SPEED PROFILES
    //  Defines how fast the robot should move in different contexts.
    //  - MATCH:   Real‑world competition speed (safe + realistic)
    //  - SIM:     High-speed simulation mode (idealized physics)
    // ============================================================
    public enum SpeedProfile {
        MATCH,
        SIM
    }

    public static final class Speed {

        // ------------------------------------------------------------
        // TELEOP SCALING (Driver Controls)
        // ------------------------------------------------------------
        // These values scale the joystick inputs before they are sent
        // to the drivetrain during teleop. They ONLY affect manual
        // driving and have no impact on autonomous commands.
        //
        // Why scale teleop input?
        //   - Prevents the robot from moving too fast during indoor
        //     testing or tight‑space debugging.
        //   - Allows full-speed control during matches.
        //   - Lets simulation run at high speed without risk.
        //
        // Translation Scale:
        //   - Controls forward/strafe speed from the left joystick.
        //   - Real Robot: kept low for safety (0.15).
        //   - Simulation: higher for convenience (0.9).
        //
        // Rotation Scale:
        //   - Controls rotational speed from the right joystick.
        //   - Real Robot: reduced for smoother control (0.25).
        //   - Simulation: higher for fast testing (0.9).
        // ------------------------------------------------------------

        // Teleop translation scaling (driver control)
        public static double getTranslationScale() {
            return edu.wpi.first.wpilibj.RobotBase.isSimulation()
                ? 1.5    // fast, convenient movement in simulation
                : 0.15;  // safe, controlled speed on the real robot
        }

        // Teleop rotation scaling (driver control)
        public static double getRotationScale() {
            return edu.wpi.first.wpilibj.RobotBase.isSimulation()
                ? 1.5    // fast rotation in simulation
                : 0.25;  // smoother, safer rotation on real hardware
        }

        // ------------------------------------------------------------
        // SPEED PROFILE SELECTION
        // ------------------------------------------------------------
        // Automatically selects the appropriate speed profile:
        //   - SIM when running in WPILib simulation
        //   - MATCH on the real robot
        // ------------------------------------------------------------
        public static final SpeedProfile PROFILE =
            edu.wpi.first.wpilibj.RobotBase.isSimulation()
                ? SpeedProfile.SIM
                : SpeedProfile.MATCH;

        // ------------------------------------------------------------
        // MAX LINEAR SPEED
        // ------------------------------------------------------------
        // MATCH_MAX_SPEED:
        //   - The real robot’s physical top speed.
        //
        // SIM_MAX_SPEED:
        //   - A higher speed used only in simulation.
        //   - Allows fast, idealized motion without real-world limits.
        // ------------------------------------------------------------
        public static final double MATCH_MAX_SPEED = 
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        public static final double SIM_MAX_SPEED = 8.0;

        // Returns the active max speed based on the current profile.
        public static double getMaxSpeed() {
            return switch (PROFILE) {
                case MATCH -> MATCH_MAX_SPEED;
                case SIM   -> SIM_MAX_SPEED;
            };
        }

        // ------------------------------------------------------------
        // MAX ANGULAR SPEED
        // ------------------------------------------------------------
        // MATCH_MAX_ANGULAR:
        //   - Rotational speed for competition.
        //
        // SIM_MAX_ANGULAR:
        //   - Very high rotational speed for simulation only.
        // ------------------------------------------------------------
        public static final double MATCH_MAX_ANGULAR =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        public static final double SIM_MAX_ANGULAR = Math.toRadians(720);

        // Returns the active angular speed based on the current profile.
        public static double getMaxAngularSpeed() {
            return switch (PROFILE) {
                case MATCH -> MATCH_MAX_ANGULAR;
                case SIM   -> SIM_MAX_ANGULAR;
            };
        }
    }

    public static final class RobotGeometry {
        public static final double HALF_LENGTH_METERS = 0.35; // half of robot length
        public static final double HALF_WIDTH_METERS  = 0.35; // half of robot width
    }
    
    public static final class IO {
        // Controllers
        public static final int JOYSTICK_PORT = 0;

        // Photon Vision
        public static final Transform3d CAMERA_TO_ROBOT =
            new Transform3d(
                new Translation3d(0.381, 0.0, 0.1905), // meters: forward, left, up
                new Rotation3d(0,0,0)     // radians: pitch, yaw, roll
            );

        // Standard deviations for measurement trust
        public static final Matrix<N3, N1> PHOTONVISION_STD_DEVS = VecBuilder.fill(
            0.02,  // 2 cm X
            0.02,  // 2 cm Y
            0.035  // ~2 degrees
        );
    }

    public static final class Shooter {
        // Motors
        public static final int SHOOTER_CAN_ID = 0;

        // MAX RPM
        public static final double MAX_RPM = 4000;
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

        // --- Deadband ---
        public static final double DEADBAND_ERROR       = 0.05; // meters, near target

        // --- Tolerances ---
        public static final double POSITION_TOLERANCE_METERS    = 0.05;
        public static final double ANGLE_TOLERANCE_RADIANS      = Math.toRadians(2.0);

        // --- Timeouts ---
        public static final double STALL_THRESHOLD  = 0.02; // meters
        public static final double STALL_TIMEOUT    = 1.0;  // seconds
        
        // --- Poses ---
        public static final Pose2d POSE_A = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(90));
        public static final Pose2d POSE_B = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0));
    }

    public static final class FieldElements {
        // --- April Tags ---
        public static final AprilTagFieldLayout APRILTAG_LAYOUT =
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

        // --- Field ---
        public static final double FIELD_LENGTH = 16.54; // meters
        public static final double FIELD_WIDTH  = 8.21;  // meters
        public static final List<Translation2d> FIELD_BOUNDARY = List.of(
            new Translation2d(0, 0),
            new Translation2d(FIELD_LENGTH, 0),
            new Translation2d(FIELD_LENGTH, FIELD_WIDTH),
            new Translation2d(0, FIELD_WIDTH)
        );

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

        // --- Hub ---
        public static final Translation2d HUB_POSE = new Translation2d(4.5, 4.0);
        public static final double HUB_CLEARANCE_Y = 1.5; // vertical margin
        public static final double HUB_CLEARANCE_X = 1.5; // lateral margin
        public static final List<Translation2d> HUB_LIST = List.of(
            new Translation2d(4.5, 5.0),
            new Translation2d(3.65, 4.4),
            new Translation2d(3.65, 3.5),
            new Translation2d(4.5, 3.0),
            new Translation2d(5.35, 3.5),
            new Translation2d(5.35, 4.4)
        );

        public static final Translation2d SHOT_A_POSE = new Translation2d(1.0, 1.0);
        public static final Translation2d SHOT_B_POSE = new Translation2d(1.0, 7.0);
    }
}
