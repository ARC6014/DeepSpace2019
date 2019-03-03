package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.DropHatch;
import frc.robot.commands.LiftElevator;
import frc.robot.commands.TimedDrive;

public class GetHatchPlace extends CommandGroup {

    public GetHatchPlace() {
        if (Robot.elevator.elevatorHeightCm() - 10 > Robot.elevator.baseToIntakeHeight) {
            addSequential(new DropHatch());
            addSequential(new TimedDrive(0, -0.8,0.3));
            addSequential(new LiftElevator(Robot.elevator.getSetpoint() + 10));
        }
    }
}
