package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DropHatch extends Command {

    public DropHatch() {

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Robot.elevator.setSetpoint(Robot.elevator.getSetpoint()-15);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void interrupted() {
        end();
    }

    @Override
    public void end() {

    }

}
