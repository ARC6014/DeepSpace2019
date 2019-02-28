package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.LiftElevator;

public class RocketHatchL2 extends CommandGroup {
    private final double hatchHeight = 119;

    public RocketHatchL2() {
        addSequential(new LiftElevator(hatchHeight));
    }
}