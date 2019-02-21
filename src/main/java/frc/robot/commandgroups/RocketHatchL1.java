package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.AutoLiftElevator;

public class RocketHatchL1 extends CommandGroup {
    private final double hatchHeight = 48;

    public RocketHatchL1() {
        addSequential(new AutoLiftElevator(hatchHeight));
    }
}
