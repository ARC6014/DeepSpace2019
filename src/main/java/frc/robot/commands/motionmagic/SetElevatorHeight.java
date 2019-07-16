package frc.robot.commands.motionmagic;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetElevatorHeight extends Command{

    private double height;

    public SetElevatorHeight(double height) {
        this.height = height;
    }

    @Override
    protected void initialize() {
        Robot.motionMagicElevator.setSetPoint(height);
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}
