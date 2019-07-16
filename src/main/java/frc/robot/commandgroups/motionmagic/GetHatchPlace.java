package frc.robot.commandgroups.motionmagic;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Delay;
import frc.robot.commands.DropHatch;
import frc.robot.commands.LiftElevator;
import frc.robot.commands.TimedDrive;
import frc.robot.commands.motionmagic.SetElevatorHeight;

public class GetHatchPlace extends CommandGroup {

    public GetHatchPlace() {
        if (Robot.motionMagicElevator.getElevatorHeight() - 5 > Robot.elevator.baseToIntakeHeight && !Robot.elevatorActive && Robot.motionMagicElevator.getSetPointHeight() > 70) {
            Robot.elevatorActive = true;
            addSequential(new DropHatch());
            addSequential(new Delay(0.3));
            addSequential(new TimedDrive(0, -0.8,0.3));
            addSequential(new Delay(0.3));
            addSequential(new Hatch1Xbox());
            Robot.elevatorActive = false;
        } else if (Robot.elevator.elevatorHeightCm() - 5 > Robot.elevator.baseToIntakeHeight && !Robot.elevatorActive) {
            addSequential(new SetElevatorHeight(Robot.elevator.baseToIntakeHeight));
            addSequential(new Delay(0.3));
            addSequential(new TimedDrive(0, -0.8,0.3));
            addSequential(new SetElevatorHeight(63));
        }
    }
}
