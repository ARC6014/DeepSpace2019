package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.LiftElevator;

public class RocketCargoL1 extends CommandGroup {
    private final double cargoHeight = 95;

    public RocketCargoL1() {
        addSequential(new LiftElevator(cargoHeight));
    }
}
