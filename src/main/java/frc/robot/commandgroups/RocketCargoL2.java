package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.LiftElevator;

public class RocketCargoL2 extends CommandGroup {
    private final double cargoHeight = 173;

    public RocketCargoL2() {
        addSequential(new LiftElevator(cargoHeight));
    }
}
