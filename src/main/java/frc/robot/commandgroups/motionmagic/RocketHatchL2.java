package frc.robot.commandgroups.motionmagic;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.motionmagic.SetElevatorHeight;

public class RocketHatchL2 extends CommandGroup {
    private final double hatchHeight = 143;

    public RocketHatchL2() {
        addSequential(new SetElevatorHeight(hatchHeight));
    }
}
