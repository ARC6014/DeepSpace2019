package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LiftHatch extends Command {

    public LiftHatch() {

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Robot.elevator.setSetpoint(Robot.elevator.getSetpoint()+22);
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
