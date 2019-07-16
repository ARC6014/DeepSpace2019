package frc.robot.commandgroups.motionmagic;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.motionmagic.SetElevatorHeight;

public class RocketCargoL2 extends CommandGroup {
    private final double cargoHeight = 173;

    public RocketCargoL2() {
        addSequential(new SetElevatorHeight(cargoHeight));
    }
}
