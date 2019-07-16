package frc.robot.commandgroups.motionmagic;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.motionmagic.SetElevatorHeight;

public class IntakeBaseLevel extends CommandGroup {

    public IntakeBaseLevel() {
        addSequential(new SetElevatorHeight(0));
    }
}
