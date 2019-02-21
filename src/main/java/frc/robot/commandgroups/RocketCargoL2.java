package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.AutoLiftElevator;

public class RocketCargoL2 extends CommandGroup {
    private final double cargoHeight = 141;

    public RocketCargoL2() {
        addSequential(new AutoLiftElevator(cargoHeight));
    }
}
