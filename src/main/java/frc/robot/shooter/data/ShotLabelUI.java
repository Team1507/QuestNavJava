package frc.robot.shooter.data;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class ShotLabelUI {

    private final SimpleWidget madeWidget;
    private final SimpleWidget missAmountWidget;

    public ShotLabelUI() {
        madeWidget = Shuffleboard.getTab("Shooter")
            .add("Shot Made", false);

        missAmountWidget = Shuffleboard.getTab("Shooter")
            .add("Miss Amount", 0.0);
    }

    public boolean getMade() {
        return madeWidget.getEntry().getBoolean(false);
    }

    public double getMissAmount() {
        return missAmountWidget.getEntry().getDouble(0.0);
    }
}
