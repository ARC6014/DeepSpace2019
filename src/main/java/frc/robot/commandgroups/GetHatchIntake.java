package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.Delay;
import frc.robot.commands.LiftHatch;
import frc.robot.commands.TimedDrive;

public class GetHatchIntake extends CommandGroup {

    public GetHatchIntake() {
        if (Robot.elevator.elevatorHeightCm() + 20 < Robot.elevator.maxHeight && !Robot.elevatorActive) {
            addSequential(new LiftHatch());
            Robot.elevatorActive = true;
            addSequential(new Delay(0.3));
            addSequential(new TimedDrive(0, -0.8,0.3));
            addSequential(new Delay(0.3));
            addSequential(new RocketHatchL1());
            Robot.elevatorActive = false;
        }
    }
}
