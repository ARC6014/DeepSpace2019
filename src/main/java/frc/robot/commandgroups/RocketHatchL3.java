package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.AutoLiftElevator;

public class RocketHatchL3 extends CommandGroup {
    private final double hatchHeight = 190;

    public RocketHatchL3() {
        addSequential(new AutoLiftElevator(hatchHeight));
    }
}
