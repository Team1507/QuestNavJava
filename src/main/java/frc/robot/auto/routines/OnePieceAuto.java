package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.auto.AutoSequence;
import frc.robot.navigation.Nodes;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class OnePieceAuto {

    public static Command build(CommandSwerveDrivetrain drivetrain) {

        return new AutoSequence(drivetrain)
            .moveTo(Nodes.Start.CENTER)
            .moveTo(Nodes.Hub.APPROACH_FRONT)
            .score()
            .moveTo(Nodes.Depot.PICKUP_LEFT)
            .intake()
            .moveTo(Nodes.Park.CENTER)
            .build();
    }
}
