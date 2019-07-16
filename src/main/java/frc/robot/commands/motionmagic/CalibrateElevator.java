package frc.robot.commands.motionmagic;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CalibrateElevator extends Command {

    public CalibrateElevator() {

    }

    @Override
    protected void initialize() {
        this.setRunWhenDisabled(false);
        requires(Robot.motionMagicElevator);
    }

    @Override
    protected void execute() {
        Robot.motionMagicElevator.calibrationMovement();
    }

    @Override
    protected boolean isFinished() {
        return Robot.motionMagicElevator.getCalibrated();
    }

    @Override
    protected void end() {

    }

    @Override
    protected void interrupted() {

    }
}
