package frc.robot.commandgroups.motionmagic;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.motionmagic.SetElevatorHeight;

public class RocketHatchL3 extends CommandGroup {
    private final double hatchHeight = 210;

    public RocketHatchL3() {
        addSequential(new SetElevatorHeight(hatchHeight));
    }
}
