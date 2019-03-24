package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import java.util.concurrent.TimeUnit;

public class Testing extends Command {
    @Override
    protected void initialize() {
        Robot.doneDrivingTest = false;
        setTimeout(Robot.dt);
    }

    @Override
    protected void execute() {
        Robot.drive.arcadeDrive(1,0);
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

    @Override
    protected void end() {
        Robot.doneDrivingTest = true;
    }

    @Override
    protected void interrupted() {
        end();
    }

    int i = 0;
}
