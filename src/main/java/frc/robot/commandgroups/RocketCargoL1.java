package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.AutoLiftElevator;

public class RocketCargoL1 extends CommandGroup {
    private final double cargoHeight = 70;

    public RocketCargoL1() {
        addSequential(new AutoLiftElevator(cargoHeight));
    }
}
