package frc.robot.commandgroups.motionmagic;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.motionmagic.SetElevatorHeight;

public class RocketHatchL1 extends CommandGroup {
    private final double hatchHeight = 63;

    public RocketHatchL1() {
        addSequential(new SetElevatorHeight(hatchHeight));
    }
}
