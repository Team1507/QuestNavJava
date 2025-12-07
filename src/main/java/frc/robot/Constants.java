// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
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
    public static final class IO {
        public static final int JOYSTICK_PORT = 0;
    }

    public static final class Drive {
        public static final double TRANSLATION_SCALE = 0.15;
        public static final double ROTATION_SCALE = 0.25;
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
}
