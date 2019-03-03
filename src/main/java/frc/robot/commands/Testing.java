package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Testing extends Command {
    @Override
    protected void initialize() {
        setTimeout(5);
    }

    @Override
    protected void execute() {
        Robot.drive.arcadeDrive(1,0);

        Robot.positions.get(0).add((Robot.drive.getLeftEncoderRev()*Robot.encoderTicksPerRev));
        Robot.positions.get(1).add((Robot.drive.getRightEncoderRev()*Robot.encoderTicksPerRev));
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

    @Override
    protected void end() {
        Robot.drive.arcadeDrive(0,0);
    }

    @Override
    protected void interrupted() {
        end();
    }

    int i = 0;
}
