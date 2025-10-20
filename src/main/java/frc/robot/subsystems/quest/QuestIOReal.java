package frc.robot.subsystems.quest;

public class QuestIOReal implements QuestIO {
  private final QuestNav questNav = new QuestNav();

  @Override
  public void updateInputs(QuestIOInputs inputs) {
    inputs.connected = questNav.isConnected();

    inputs.uncorrectedPose = questNav.getUncorrectedPose();
    inputs.uncorrectedResetPose = questNav.getUncorrectedResetPose();
    inputs.uncorrectedResetToQuest = inputs.uncorrectedPose.minus(inputs.uncorrectedResetPose);
  }
}