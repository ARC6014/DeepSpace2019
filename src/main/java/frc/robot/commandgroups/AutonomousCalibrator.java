package frc.robot.commandgroups;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.commands.Calibrator;


public class AutonomousCalibrator extends CommandGroup{
    public AutonomousCalibrator () {
        addSequential(new Calibrator());
    }
}
