package frc.robot.subsystems.quest;

import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public interface QuestIO extends AutoCloseable {
  public static class QuestIOInputs {
    public boolean connected = false;

    /** Current QuestNav pose */
    public Pose2d uncorrectedPose = Pose2d.kZero;
    /** QuestNav pose when robot code started */
    public Pose2d uncorrectedResetPose = Pose2d.kZero;
    /** Transform between QuestNav current and starting pose */
    public Transform2d uncorrectedResetToQuest = Transform2d.kZero;

    public OptionalDouble timestamp = OptionalDouble.empty();
    public OptionalDouble timestampDelta = OptionalDouble.empty();
    public OptionalInt batteryLevel = OptionalInt.empty();
  }

  public default void updateInputs(QuestIOInputs inputs) {}

  public default void zeroPosition() {}

  public default void zeroHeading() {}

  @Override
  public default void close() {}
}