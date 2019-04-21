package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.LiftElevator;

public class RocketHatchL3 extends CommandGroup {
    private final double hatchHeight = 210;

    public RocketHatchL3() {
        addSequential(new LiftElevator(hatchHeight));
    }
}
