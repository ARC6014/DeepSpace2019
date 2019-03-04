package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

public class FreeElevator extends TimedCommand {

    public FreeElevator(double timeout) {
        super(timeout);
        requires(Robot.elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Robot.elevator.setElevatorSpeedManual(0);
    }

    @Override
    public void interrupted() {
        end();
    }

    @Override
    public void end() {
        Robot.elevator.setElevatorSpeedManual(0);
    }

}
