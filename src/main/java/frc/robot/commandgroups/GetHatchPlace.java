package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Delay;
import frc.robot.commands.DropHatch;
import frc.robot.commands.LiftElevator;
import frc.robot.commands.TimedDrive;

public class GetHatchPlace extends CommandGroup {

    public GetHatchPlace() {
        if (Robot.elevator.elevatorHeightCm() - 5 > Robot.elevator.baseToIntakeHeight && !Robot.elevatorActive) {
            Robot.elevatorActive = true;
            addSequential(new DropHatch());
            addSequential(new Delay(0.3));
            addSequential(new TimedDrive(0, -0.8,0.3));
            addSequential(new Delay(0.3));
            addSequential(new RocketHatchL1());
            Robot.elevatorActive = false;
        }
    }
}
