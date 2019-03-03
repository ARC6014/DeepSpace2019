package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

public class TimedDrive extends TimedCommand {

    double x;
    double y;

    public TimedDrive(double x, double y, double timeout) {
        super(timeout);
        this.x = x;
        this.y = y;
    }

    protected void initialize() {

    }

    public void execute() {
        Robot.drive.arcadeDrive(y,x);
    }

    public void interrupted() {
        end();
    }

    public void end() {
        Robot.drive.arcadeDrive(0,0);
    }
}
