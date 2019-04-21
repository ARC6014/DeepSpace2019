package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.LiftElevator;

public class RocketHatchL1 extends CommandGroup {
    private final double hatchHeight = 63;

    public RocketHatchL1() {
        addSequential(new LiftElevator(hatchHeight));
    }
}
