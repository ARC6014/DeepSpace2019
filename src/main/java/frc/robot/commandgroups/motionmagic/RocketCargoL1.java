package frc.robot.commandgroups.motionmagic;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.motionmagic.SetElevatorHeight;

public class RocketCargoL1 extends CommandGroup {
    private final double cargoHeight = 98;

    public RocketCargoL1() {
            addSequential(new SetElevatorHeight(cargoHeight));
    }
}

