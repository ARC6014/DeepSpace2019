package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.FreeElevator;
import frc.robot.commands.WaitLiftElevator;

public class IntakeBaseLevel extends CommandGroup {

    public IntakeBaseLevel() {
        addSequential(new WaitLiftElevator(Robot.elevator.baseToIntakeHeight - 1.2));
        addSequential(new FreeElevator(0.2));
    }
}
