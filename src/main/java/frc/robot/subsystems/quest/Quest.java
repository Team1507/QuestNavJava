package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;

public class Quest {
  private final QuestIO io;

  public boolean isPoseReset = false;

  private final Alert disconnectedAlert = new Alert("Quest Disconnected!", AlertType.kWarning);
  private final Alert lowBatteryAlert =
      new Alert("Quest Battery is Low! (<25%)", AlertType.kWarning);

  private Pose2d fieldToRobotOrigin = Pose2d.kZero;

  public Quest(QuestIO io) {
    this.io = io;
  }

}