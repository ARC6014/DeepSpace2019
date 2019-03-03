package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.LiftElevator;

public class IntakeBaseLevel extends CommandGroup {

    public IntakeBaseLevel() {
        addSequential(new LiftElevator(Robot.elevator.baseToIntakeHeight + 3));
    }
}
