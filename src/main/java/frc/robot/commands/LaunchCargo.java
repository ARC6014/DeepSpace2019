package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

public class LaunchCargo extends TimedCommand {

    public LaunchCargo() {
        super(0.5);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Robot.cargoIntake.setIntakeSpeed(1); //TODO check the rotation
    }

    @Override
    public void interrupted() {
        end();
    }

    @Override
    public void end() {
        Robot.cargoIntake.setIntakeSpeed(0);
    }

}
