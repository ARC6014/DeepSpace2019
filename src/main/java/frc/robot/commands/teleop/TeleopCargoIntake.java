package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TeleopCargoIntake  extends Command{
    private double power;

    public TeleopCargoIntake() {}

    @Override
    protected void initialize() {}

    @Override
    protected void execute() {
        Robot.cargoIntake.setIntakeSpeed(power);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Robot.cargoIntake.setIntakeSpeed(0);
    }

    @Override
    protected void interrupted() { end(); }
}
