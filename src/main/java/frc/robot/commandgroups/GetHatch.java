package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.LiftHatch;
import frc.robot.commands.TimedDrive;

public class GetHatch extends CommandGroup {

    public GetHatch() {
        if (Robot.elevator.elevatorHeightCm() + 20 < Robot.elevator.maxHeight) {

        } else {
            addSequential(new LiftHatch());
        }
        addSequential(new TimedDrive(0, -0.8,0.3));
    }
}
